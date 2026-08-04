#include "camera/dmabuf_cpu_sync.h"
#include "camera/v4l2_camera.h"
#include "camera/v4l2_dmabuf_contract.h"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string device;
    uint32_t width = 0U;
    uint32_t height = 0U;
    uint32_t pixel_format = 0U;
    uint32_t fps = 0U;
    uint32_t buffers = 6U;
    uint32_t frames = 120U;
    int timeout_ms = 2000;
    bool prefer_multiplanar = false;
    bool require_all_buffers = true;
    std::string report_path;
};

uint32_t ParseUint32(const std::string& text, const char* name) {
    std::size_t consumed = 0U;
    const unsigned long value = std::stoul(text, &consumed, 10);
    if (consumed != text.size() || value > 0xffffffffUL) {
        throw std::invalid_argument(std::string("invalid ") + name);
    }
    return static_cast<uint32_t>(value);
}

int ParseInt(const std::string& text, const char* name) {
    std::size_t consumed = 0U;
    const long value = std::stol(text, &consumed, 10);
    if (consumed != text.size() || value <= 0 || value > 60000L) {
        throw std::invalid_argument(std::string("invalid ") + name);
    }
    return static_cast<int>(value);
}

void PrintUsage(const char* program) {
    std::cerr
        << "Usage: " << program << " --device /dev/videoX --width N --height N"
        << " --format NV12|NM12 --fps N [options]\n"
        << "Options:\n"
        << "  --buffers N             requested V4L2 buffer count (default 6)\n"
        << "  --frames N              frames to validate (default 120)\n"
        << "  --timeout-ms N          capture poll timeout (default 2000)\n"
        << "  --prefer-mplane         prefer VIDEO_CAPTURE_MPLANE\n"
        << "  --allow-unseen-buffers  do not fail if not every index is observed\n"
        << "  --report PATH           write text report\n";
}

Options ParseOptions(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::invalid_argument(std::string(name) + " needs a value");
            }
            return argv[++index];
        };

        if (arg == "--device") {
            options.device = require_value("--device");
        } else if (arg == "--width") {
            options.width = ParseUint32(require_value("--width"), "width");
        } else if (arg == "--height") {
            options.height = ParseUint32(require_value("--height"), "height");
        } else if (arg == "--format") {
            options.pixel_format =
                visionarm::FourccFromString(require_value("--format"));
        } else if (arg == "--fps") {
            options.fps = ParseUint32(require_value("--fps"), "fps");
        } else if (arg == "--buffers") {
            options.buffers = ParseUint32(require_value("--buffers"), "buffers");
        } else if (arg == "--frames") {
            options.frames = ParseUint32(require_value("--frames"), "frames");
        } else if (arg == "--timeout-ms") {
            options.timeout_ms = ParseInt(
                require_value("--timeout-ms"), "timeout-ms");
        } else if (arg == "--prefer-mplane") {
            options.prefer_multiplanar = true;
        } else if (arg == "--allow-unseen-buffers") {
            options.require_all_buffers = false;
        } else if (arg == "--report") {
            options.report_path = require_value("--report");
        } else if (arg == "--help" || arg == "-h") {
            PrintUsage(argv[0]);
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument("unknown option: " + arg);
        }
    }

    if (options.device.empty() || options.width == 0U ||
        options.height == 0U || options.pixel_format == 0U ||
        options.fps == 0U || options.buffers < 2U || options.frames == 0U) {
        throw std::invalid_argument("required options are missing or invalid");
    }
    return options;
}

void EmitInventory(
    std::ostream& output,
    const visionarm::V4L2DmabufInventory& inventory) {
    output << "accepted_format="
           << inventory.width << 'x' << inventory.height
           << " fourcc=" << visionarm::FourccToString(inventory.pixel_format)
           << " api=" << (inventory.multiplanar_api ? "mplane" : "single")
           << " planes=" << inventory.plane_count
           << " buffers=" << inventory.buffers.size() << '\n';
    output << "colorspace=" << inventory.colorspace
           << " ycbcr=" << inventory.ycbcr_encoding
           << " quantization=" << inventory.quantization
           << " xfer=" << inventory.transfer_function << '\n';

    for (const auto& buffer : inventory.buffers) {
        for (const auto& plane : buffer.planes) {
            output << "buffer=" << plane.buffer_index
                   << " plane=" << plane.plane_index
                   << " fd=" << plane.dma_fd
                   << " mmap_length=" << plane.mmap_length
                   << " stride=" << plane.stride
                   << " size_image=" << plane.size_image
                   << " cloexec=" << (plane.fd_cloexec ? 1 : 0)
                   << " size_query=" << (plane.size_query_ok ? 1 : 0)
                   << " fd_size=" << plane.fd_size
                   << " dev=" << plane.device_id
                   << " inode=" << plane.inode << '\n';
        }
    }
}

int Run(const Options& options) {
    visionarm::V4L2CameraConfig config;
    config.device = options.device;
    config.width = options.width;
    config.height = options.height;
    config.pixel_format = options.pixel_format;
    config.fps = options.fps;
    config.buffer_count = options.buffers;
    config.timeout_ms = options.timeout_ms;
    config.nonblocking = true;
    config.prefer_multiplanar = options.prefer_multiplanar;
    config.export_dmabuf = true;
    config.require_dmabuf_export = true;

    visionarm::V4L2Camera camera(config);
    camera.Open();

    const visionarm::V4L2DmabufInventory inventory =
        camera.GetDmabufInventory();
    const auto inventory_result =
        visionarm::ValidateV4L2DmabufInventory(inventory);

    std::ofstream report;
    if (!options.report_path.empty()) {
        report.open(options.report_path, std::ios::out | std::ios::trunc);
        if (!report) {
            throw std::runtime_error("cannot open report: " + options.report_path);
        }
    }

    EmitInventory(std::cout, inventory);
    if (report) {
        EmitInventory(report, inventory);
    }

    if (!inventory_result.ok) {
        std::cerr << "inventory_contract=FAIL error="
                  << visionarm::ToString(inventory_result.error)
                  << " buffer=" << inventory_result.buffer_index
                  << " plane=" << inventory_result.plane_index
                  << " expected=" << inventory_result.expected_minimum
                  << " actual=" << inventory_result.actual << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "inventory_contract=PASS\n";

    camera.Start();
    std::set<uint32_t> observed_buffers;
    uint64_t checksum = 0U;
    uint32_t validated_frames = 0U;
    uint32_t timeouts = 0U;
    uint32_t dropped = 0U;

    while (validated_frames < options.frames) {
        visionarm::CaptureFrameView frame;
        const visionarm::CaptureResult result = camera.Capture(&frame);
        if (result == visionarm::CaptureResult::TIMEOUT ||
            result == visionarm::CaptureResult::WAKE) {
            ++timeouts;
            if (timeouts > options.frames * 4U) {
                std::cerr << "too many capture timeouts\n";
                camera.Stop();
                return EXIT_FAILURE;
            }
            continue;
        }
        if (result == visionarm::CaptureResult::DROPPED) {
            ++dropped;
            continue;
        }
        if (result == visionarm::CaptureResult::STOPPED) {
            std::cerr << "camera stopped before probe completed\n";
            camera.Stop();
            return EXIT_FAILURE;
        }

        const auto frame_result = visionarm::ValidateNv12DmabufFrame(frame);
        if (!frame_result.ok) {
            std::cerr << "frame_contract=FAIL frame=" << frame.identity.frame_id
                      << " error=" << visionarm::ToString(frame_result.error)
                      << " buffer=" << frame_result.buffer_index
                      << " plane=" << frame_result.plane_index
                      << " expected=" << frame_result.expected_minimum
                      << " actual=" << frame_result.actual << '\n';
            (void)camera.Requeue(visionarm::MakeRequeueRequest(frame));
            camera.Stop();
            return EXIT_FAILURE;
        }

        visionarm::DmabufCpuAccessGuard cpu_guard;
        int failed_fd = -1;
        int sync_errno = 0;
        if (!cpu_guard.Begin(
                frame,
                visionarm::DmabufCpuAccessMode::READ,
                &failed_fd,
                &sync_errno)) {
            std::cerr << "dmabuf_sync_start=FAIL fd=" << failed_fd
                      << " errno=" << sync_errno << '\n';
            (void)camera.Requeue(visionarm::MakeRequeueRequest(frame));
            camera.Stop();
            return EXIT_FAILURE;
        }

        for (uint32_t plane_index = 0U;
             plane_index < frame.plane_count;
             ++plane_index) {
            const auto* bytes = static_cast<const uint8_t*>(
                frame.planes[plane_index].mapped_address);
            const std::size_t sample = std::min<std::size_t>(
                frame.planes[plane_index].bytes_used,
                64U);
            for (std::size_t byte_index = 0U;
                 byte_index < sample;
                 ++byte_index) {
                checksum = (checksum * 131U) + bytes[byte_index];
            }
        }

        if (!cpu_guard.End(&failed_fd, &sync_errno)) {
            std::cerr << "dmabuf_sync_end=FAIL fd=" << failed_fd
                      << " errno=" << sync_errno << '\n';
            (void)camera.Requeue(visionarm::MakeRequeueRequest(frame));
            camera.Stop();
            return EXIT_FAILURE;
        }

        observed_buffers.insert(frame.buffer_index);
        if (!camera.Requeue(visionarm::MakeRequeueRequest(frame))) {
            std::cerr << "key_aware_requeue=FAIL frame="
                      << frame.identity.frame_id
                      << " buffer=" << frame.buffer_index << '\n';
            camera.Stop();
            return EXIT_FAILURE;
        }
        ++validated_frames;
    }

    const uint32_t outstanding = camera.outstanding_buffers();
    camera.Stop();

    const bool all_buffers_seen =
        observed_buffers.size() == inventory.buffers.size();
    std::cout << "frame_contract=PASS frames=" << validated_frames
              << " dropped=" << dropped
              << " timeouts=" << timeouts
              << " observed_buffers=" << observed_buffers.size()
              << '/' << inventory.buffers.size()
              << " checksum=" << checksum
              << " outstanding_before_stop=" << outstanding << '\n';
    std::cout << "dmabuf_cpu_sync=PASS\n";
    std::cout << "key_aware_requeue=PASS\n";

    if (report) {
        report << "validated_frames=" << validated_frames << '\n'
               << "dropped=" << dropped << '\n'
               << "timeouts=" << timeouts << '\n'
               << "observed_buffers=" << observed_buffers.size() << '\n'
               << "allocated_buffers=" << inventory.buffers.size() << '\n'
               << "checksum=" << checksum << '\n'
               << "outstanding_before_stop=" << outstanding << '\n';
    }

    if (outstanding != 0U) {
        std::cerr << "outstanding buffer count is not zero\n";
        return EXIT_FAILURE;
    }
    if (options.require_all_buffers && !all_buffers_seen) {
        std::cerr << "not every allocated V4L2 buffer index was observed\n";
        return EXIT_FAILURE;
    }

    std::cout << "v4l2_dmabuf_probe PASSED\n";
    return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = ParseOptions(argc, argv);
        return Run(options);
    } catch (const std::exception& error) {
        std::cerr << "v4l2_dmabuf_probe failed: " << error.what() << '\n';
        PrintUsage(argv[0]);
        return EXIT_FAILURE;
    }
}
