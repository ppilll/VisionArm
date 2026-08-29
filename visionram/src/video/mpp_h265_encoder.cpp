#define MODULE_TAG "visionarm_mpp_h265_encoder"

#include "video/mpp_h265_encoder.h"

#include "observability/logger.h"

#include <algorithm>
#include <cstdint>
#include <rk_venc_cfg.h>
#include <mpp_meta.h>
#include <mpp_packet.h>
#include <rk_mpi_cmd.h>
#include <stdexcept>
#include <utility>

namespace visionarm {
namespace {

[[nodiscard]] bool MppOk(MPP_RET result, const char* operation) noexcept {
    if (result == MPP_OK) {
        return true;
    }
    logging::Log(logging::LogLevel::ERROR, "mpp", operation,
                 " failed, ret=", result);
    return false;
}

[[nodiscard]] std::size_t DefaultPacketCapacity(
    const MppH265EncoderConfig& config) noexcept {
    const uint64_t raw =
        static_cast<uint64_t>(std::max(config.horizontal_stride, config.width)) *
        static_cast<uint64_t>(std::max(config.vertical_stride, config.height)) *
        3U / 2U;
    const uint64_t one_megabyte = 1024U * 1024U;
    return static_cast<std::size_t>(std::max(raw, one_megabyte));
}

[[nodiscard]] int64_t NominalFrameDurationUs(
    const MppH265EncoderConfig& config) noexcept {
    const int64_t numerator =
        static_cast<int64_t>(config.fps_denominator) * 1'000'000LL;
    return (numerator + config.fps_numerator / 2) /
           static_cast<int64_t>(config.fps_numerator);
}

[[nodiscard]] bool IsHevcRandomAccessPicture(
    const uint8_t* data,
    std::size_t size) noexcept {
    if (data == nullptr || size < 6U) {
        return false;
    }

    std::size_t offset = 0U;
    while (offset + 4U < size) {
        std::size_t start = size;
        std::size_t prefix = 0U;
        for (std::size_t i = offset; i + 3U < size; ++i) {
            if (data[i] == 0U && data[i + 1U] == 0U && data[i + 2U] == 1U) {
                start = i;
                prefix = 3U;
                break;
            }
            if (i + 4U < size && data[i] == 0U && data[i + 1U] == 0U &&
                data[i + 2U] == 0U && data[i + 3U] == 1U) {
                start = i;
                prefix = 4U;
                break;
            }
        }
        if (start == size || start + prefix >= size) {
            break;
        }

        const uint8_t nal_header = data[start + prefix];
        const uint8_t nal_type = static_cast<uint8_t>((nal_header >> 1U) & 0x3FU);
        // HEVC IRAP VCL NAL unit types BLA/IDR/CRA are 16..21.
        if (nal_type >= 16U && nal_type <= 21U) {
            return true;
        }
        offset = start + prefix + 2U;
    }
    return false;
}

}  // namespace

MppH265Encoder::~MppH265Encoder() {
    Shutdown();
}

void MppH265Encoder::Initialize(const MppH265EncoderConfig& config) {
    Shutdown();
    snapshot_ = {};
    if (config.width <= 0 || config.height <= 0 ||
        config.horizontal_stride < config.width ||
        config.vertical_stride < config.height ||
        config.fps_numerator <= 0 || config.fps_denominator <= 0 ||
        config.bitrate_bps <= 0 || config.gop_length <= 0 ||
        config.max_source_buffers == 0U) {
        throw std::invalid_argument("invalid MPP H.265 encoder config");
    }
    // The frozen sensor capability matrix contains integer frame rates
    // only (30/60/90 fps), so the media-side configured FPS denominator is 1.
    // Do not try vendor-specific MppEncCfg key spellings at runtime: some
    // vendor librockchip_mpp builds do not safely tolerate unknown keys.
    // MPP_ENC_GET_CFG initializes both FPS denominators to the runtime default
    // (1 on supported Rockchip MPP releases), therefore only the numerator
    // needs to be overridden for this product.
    if (config.fps_denominator != 1) {
        throw std::invalid_argument(
            "MPP H.265 encoder supports only integer FPS on this BSP");
    }

    config_ = config;
    if (config_.packet_buffer_bytes == 0U) {
        config_.packet_buffer_bytes = DefaultPacketCapacity(config_);
    }

    logging::Log(logging::LogLevel::DEBUG, "mpp", "init stage=create");
    if (!MppOk(mpp_create(&context_, &mpi_), "mpp_create") ||
        context_ == nullptr || mpi_ == nullptr) {
        Shutdown();
        throw std::runtime_error("failed to create MPP encoder context");
    }

    logging::Log(logging::LogLevel::DEBUG, "mpp",
                 "init stage=context_created");
    MppPollType timeout = MPP_POLL_BLOCK;
    if (!MppOk(
            mpi_->control(context_, MPP_SET_OUTPUT_TIMEOUT, &timeout),
            "MPP_SET_OUTPUT_TIMEOUT") ||
        !MppOk(
            mpp_init(context_, MPP_CTX_ENC, MPP_VIDEO_CodingHEVC),
            "mpp_init(HEVC)")) {
        Shutdown();
        throw std::runtime_error("failed to initialize MPP HEVC encoder");
    }

    logging::Log(logging::LogLevel::DEBUG, "mpp", "init stage=configure");
    if (!ConfigureEncoder()) {
        Shutdown();
        throw std::runtime_error("failed to configure MPP HEVC encoder");
    }

    logging::Log(logging::LogLevel::DEBUG, "mpp", "init stage=configured");
    if (!MppOk(
            mpp_buffer_group_get_internal(
                &packet_group_, MPP_BUFFER_TYPE_DRM),
            "mpp_buffer_group_get_internal") ||
        !MppOk(
            mpp_buffer_get(
                packet_group_, &packet_buffer_,
                config_.packet_buffer_bytes),
            "mpp_buffer_get(packet)")) {
        Shutdown();
        throw std::runtime_error("failed to allocate MPP packet buffer");
    }

    imported_sources_.resize(config_.max_source_buffers);
    logging::Log(logging::LogLevel::DEBUG, "mpp", "init stage=codec_header");
    if (!BuildCodecHeader()) {
        Shutdown();
        throw std::runtime_error("failed to obtain H.265 VPS/SPS/PPS");
    }
    initialized_ = true;
    logging::Log(logging::LogLevel::INFO, "mpp", "encoder ready");
}

bool MppH265Encoder::ConfigureEncoder() noexcept {
    MppEncCfg cfg = nullptr;
    if (!MppOk(mpp_enc_cfg_init(&cfg), "mpp_enc_cfg_init") || cfg == nullptr) {
        return false;
    }

    bool ok = MppOk(
        mpi_->control(context_, MPP_ENC_GET_CFG, cfg),
        "MPP_ENC_GET_CFG");

    auto set = [&ok, cfg](const char* key, int value) {
        if (ok && mpp_enc_cfg_set_s32(cfg, key, value) != MPP_OK) {
            logging::Log(logging::LogLevel::ERROR, "mpp",
                         "mpp_enc_cfg_set_s32 failed for ", key);
            ok = false;
        }
    };

    set("codec:type", MPP_VIDEO_CodingHEVC);
    set("prep:width", config_.width);
    set("prep:height", config_.height);
    set("prep:hor_stride", config_.horizontal_stride);
    set("prep:ver_stride", config_.vertical_stride);
    set("prep:format", MPP_FMT_YUV420SP);

    set("rc:mode", MPP_ENC_RC_MODE_CBR);
    set("rc:fps_in_flex", 0);
    set("rc:fps_in_num", config_.fps_numerator);
    // Keep the denominator returned by MPP_ENC_GET_CFG.  The product only
    // uses integer FPS and the Rockchip MPP default denominator is 1.
    set("rc:fps_out_flex", 0);
    set("rc:fps_out_num", config_.fps_numerator);
    set("rc:gop", config_.gop_length);
    set("rc:bps_target", config_.bitrate_bps);
    set("rc:bps_max", config_.bitrate_bps * 17 / 16);
    set("rc:bps_min", config_.bitrate_bps * 15 / 16);
    set("rc:qp_init", -1);
    set("rc:qp_min", config_.qp_min);
    set("rc:qp_max", config_.qp_max);
    set("rc:qp_min_i", config_.qp_min_i);
    set("rc:qp_max_i", config_.qp_max_i);
    // 当前预编译 MPP 不支持这个配置键。
    // 使用 MPP_ENC_GET_CFG 返回的默认值。
    // set("h265:diff_cu_qp_delta_depth", 0);

    if (ok) {
        ok = MppOk(
            mpi_->control(context_, MPP_ENC_SET_CFG, cfg),
            "MPP_ENC_SET_CFG");
    }
    mpp_enc_cfg_deinit(cfg);
    return ok;
}

bool MppH265Encoder::BuildCodecHeader() noexcept {
    codec_config_packets_.clear();
    MppPacket packet = nullptr;
    if (!MppOk(
            mpp_packet_init_with_buffer(&packet, packet_buffer_),
            "mpp_packet_init_with_buffer(header)")) {
        return false;
    }
    mpp_packet_set_length(packet, 0U);
    const MPP_RET result =
        mpi_->control(context_, MPP_ENC_GET_HDR_SYNC, packet);
    if (!MppOk(result, "MPP_ENC_GET_HDR_SYNC")) {
        mpp_packet_deinit(&packet);
        return false;
    }

    const auto* data = static_cast<const uint8_t*>(mpp_packet_get_pos(packet));
    const std::size_t size = mpp_packet_get_length(packet);
    if (data == nullptr || size == 0U) {
        mpp_packet_deinit(&packet);
        return false;
    }

    EncodedPacket header;
    header.codec_config = true;
    header.bytes.assign(data, data + size);
    codec_config_packets_.push_back(std::move(header));
    ++snapshot_.codec_config_packets;
    mpp_packet_deinit(&packet);
    return true;
}

bool MppH265Encoder::EnsureSourceImported(
    const CaptureFrameView& frame,
    MppBuffer* buffer) noexcept {
    if (buffer == nullptr || frame.buffer_index >= imported_sources_.size()) {
        return false;
    }

    Nv12MppLayout layout;
    if (!DeriveNv12MppLayout(frame, config_.vertical_stride, &layout) ||
        layout.width != config_.width || layout.height != config_.height ||
        layout.horizontal_stride != config_.horizontal_stride ||
        layout.vertical_stride != config_.vertical_stride) {
        return false;
    }

    const CapturePlaneView& plane = frame.planes[0];
    ImportedSource& entry = imported_sources_[frame.buffer_index];
    if (entry.buffer != nullptr && entry.fd == plane.dma_fd &&
        entry.size == plane.allocation_length) {
        *buffer = entry.buffer;
        return true;
    }

    if (entry.buffer != nullptr) {
        mpp_buffer_put(entry.buffer);
        entry = {};
        ++snapshot_.source_buffer_reimports;
    }

    MppBufferInfo info{};
    info.type = MPP_BUFFER_TYPE_EXT_DMA;
    info.size = plane.allocation_length;
    info.ptr = nullptr;
    info.hnd = nullptr;
    info.fd = plane.dma_fd;
    info.index = static_cast<int>(frame.buffer_index);

    if (!MppOk(
            mpp_buffer_import(&entry.buffer, &info),
            "mpp_buffer_import(camera dmabuf)")) {
        entry = {};
        return false;
    }
    entry.fd = plane.dma_fd;
    entry.size = plane.allocation_length;
    ++snapshot_.imported_source_buffers;
    *buffer = entry.buffer;
    return true;
}

bool MppH265Encoder::EncodeOne(
    const CaptureFrameView& frame,
    MppBuffer source,
    std::vector<EncodedPacket>* packets) noexcept {
    MppFrame mpp_frame = nullptr;
    MppPacket packet = nullptr;
    if (!MppOk(mpp_frame_init(&mpp_frame), "mpp_frame_init") ||
        !MppOk(
            mpp_packet_init_with_buffer(&packet, packet_buffer_),
            "mpp_packet_init_with_buffer(frame)")) {
        if (mpp_frame != nullptr) {
            mpp_frame_deinit(&mpp_frame);
        }
        if (packet != nullptr) {
            mpp_packet_deinit(&packet);
        }
        return false;
    }

    mpp_packet_set_length(packet, 0U);
    mpp_frame_set_width(mpp_frame, config_.width);
    mpp_frame_set_height(mpp_frame, config_.height);
    mpp_frame_set_hor_stride(mpp_frame, config_.horizontal_stride);
    mpp_frame_set_ver_stride(mpp_frame, config_.vertical_stride);
    mpp_frame_set_fmt(mpp_frame, MPP_FMT_YUV420SP);
    mpp_frame_set_buffer(mpp_frame, source);
    mpp_frame_set_buf_size(mpp_frame, frame.planes[0].allocation_length);

    int64_t video_pts_ns = frame.identity.capture_timestamp_ns;
    if (config_.media_epoch_monotonic_ns > 0) {
        if (video_pts_ns < config_.media_epoch_monotonic_ns) {
            mpp_frame_deinit(&mpp_frame);
            mpp_packet_deinit(&packet);
            return false;
        }
        video_pts_ns -= config_.media_epoch_monotonic_ns;
    }
    mpp_frame_set_pts(
        mpp_frame,
        static_cast<RK_S64>(video_pts_ns / 1000));

    MppMeta meta = mpp_frame_get_meta(mpp_frame);
    if (meta == nullptr ||
        mpp_meta_set_packet(meta, KEY_OUTPUT_PACKET, packet) != MPP_OK) {
        mpp_frame_deinit(&mpp_frame);
        mpp_packet_deinit(&packet);
        return false;
    }

    if (!MppOk(
            mpi_->encode_put_frame(context_, mpp_frame),
            "encode_put_frame")) {
        mpp_frame_deinit(&mpp_frame);
        mpp_packet_deinit(&packet);
        return false;
    }
    mpp_frame_deinit(&mpp_frame);

    bool end_of_image = false;
    do {
        if (!MppOk(
                mpi_->encode_get_packet(context_, &packet),
                "encode_get_packet") ||
            packet == nullptr) {
            if (packet != nullptr) {
                mpp_packet_deinit(&packet);
            }
            return false;
        }

        const auto* data =
            static_cast<const uint8_t*>(mpp_packet_get_pos(packet));
        const std::size_t size = mpp_packet_get_length(packet);
        if (data == nullptr || size == 0U ||
            size > config_.packet_buffer_bytes) {
            mpp_packet_deinit(&packet);
            return false;
        }

        const bool partition = mpp_packet_is_partition(packet) != 0U;
        end_of_image = !partition || mpp_packet_is_eoi(packet) != 0U;

        EncodedPacket encoded;
        encoded.identity = frame.identity;
        const RK_S64 packet_pts = mpp_packet_get_pts(packet);
        const RK_S64 packet_dts = mpp_packet_get_dts(packet);
        encoded.pts_us = packet_pts >= 0 ?
            static_cast<int64_t>(packet_pts) : video_pts_ns / 1000;
        encoded.dts_us = packet_dts >= 0 ?
            static_cast<int64_t>(packet_dts) : encoded.pts_us;
        encoded.duration_us = NominalFrameDurationUs(config_);
        encoded.keyframe = IsHevcRandomAccessPicture(data, size);
        encoded.end_of_frame = end_of_image;
        encoded.end_of_stream = mpp_packet_get_eos(packet) != 0U;
        encoded.bytes.assign(data, data + size);
        packets->push_back(std::move(encoded));
        mpp_packet_deinit(&packet);
    } while (!end_of_image);

    return true;
}

bool MppH265Encoder::Encode(
    const CaptureFrameView& frame,
    std::vector<EncodedPacket>* packets) noexcept {
    if (!initialized_ || packets == nullptr) {
        return false;
    }
    packets->clear();
    ++snapshot_.submitted_frames;

    MppBuffer source = nullptr;
    if (!EnsureSourceImported(frame, &source) ||
        !EncodeOne(frame, source, packets)) {
        ++snapshot_.encode_failures;
        packets->clear();
        return false;
    }

    ++snapshot_.encoded_frames;
    snapshot_.emitted_packets += packets->size();
    for (const EncodedPacket& packet : *packets) {
        snapshot_.emitted_bytes += packet.bytes.size();
    }
    return true;
}

void MppH265Encoder::Shutdown() noexcept {
    initialized_ = false;
    codec_config_packets_.clear();
    for (ImportedSource& source : imported_sources_) {
        if (source.buffer != nullptr) {
            mpp_buffer_put(source.buffer);
        }
        source = {};
    }
    imported_sources_.clear();

    if (packet_buffer_ != nullptr) {
        mpp_buffer_put(packet_buffer_);
        packet_buffer_ = nullptr;
    }
    if (packet_group_ != nullptr) {
        mpp_buffer_group_put(packet_group_);
        packet_group_ = nullptr;
    }
    if (context_ != nullptr && mpi_ != nullptr) {
        (void)mpi_->reset(context_);
    }
    if (context_ != nullptr) {
        mpp_destroy(context_);
        context_ = nullptr;
    }
    mpi_ = nullptr;
}

}  // namespace visionarm
