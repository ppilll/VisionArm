#include "camera/v4l2_dmabuf_contract.h"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <linux/videodev2.h>

namespace {

#define CHECK_TRUE(expr)                                                     \
    do {                                                                     \
        if (!(expr)) {                                                       \
            std::cerr << "CHECK failed at " << __FILE__ << ':' << __LINE__   \
                      << ": " #expr "\n";                                  \
            return false;                                                    \
        }                                                                    \
    } while (false)

visionarm::CaptureFrameView MakeNv12(
    std::array<uint8_t, 24U>* bytes) {
    visionarm::CaptureFrameView frame;
    frame.identity.capture_session_id = 1U;
    frame.identity.frame_id = 1U;
    frame.buffer_index = 0U;
    frame.width = 4;
    frame.height = 4;
    frame.pixel_format = V4L2_PIX_FMT_NV12;
    frame.plane_count = 1U;
    frame.planes[0].mapped_address = bytes->data();
    frame.planes[0].dma_fd = 3;
    frame.planes[0].bytes_used = 24U;
    frame.planes[0].allocation_length = 24U;
    frame.planes[0].stride = 4U;
    frame.planes[0].size_image = 24U;
    return frame;
}

bool TestNv12Valid() {
    std::array<uint8_t, 24U> bytes{};
    const auto result = visionarm::ValidateNv12DmabufFrame(MakeNv12(&bytes));
    CHECK_TRUE(result.ok);
    return true;
}

bool TestNv12StrideFailure() {
    std::array<uint8_t, 24U> bytes{};
    auto frame = MakeNv12(&bytes);
    frame.planes[0].stride = 2U;
    const auto result = visionarm::ValidateNv12DmabufFrame(frame);
    CHECK_TRUE(!result.ok);
    CHECK_TRUE(result.error ==
               visionarm::V4L2DmabufContractError::STRIDE_TOO_SMALL);
    return true;
}

bool TestNv12MValid() {
    std::array<uint8_t, 16U> y{};
    std::array<uint8_t, 8U> uv{};
    visionarm::CaptureFrameView frame;
    frame.identity.capture_session_id = 1U;
    frame.identity.frame_id = 2U;
    frame.buffer_index = 1U;
    frame.width = 4;
    frame.height = 4;
    frame.pixel_format = V4L2_PIX_FMT_NV12M;
    frame.plane_count = 2U;
    frame.planes[0] = {y.data(), 3, 0U, 16U, 16U, 4U, 16U};
    frame.planes[1] = {uv.data(), 4, 0U, 8U, 8U, 4U, 8U};
    CHECK_TRUE(visionarm::ValidateNv12DmabufFrame(frame).ok);
    return true;
}

bool TestInventory() {
    visionarm::V4L2DmabufInventory inventory;
    inventory.plane_count = 1U;
    visionarm::V4L2DmabufBufferInventory buffer;
    buffer.buffer_index = 0U;
    visionarm::V4L2DmabufPlaneInventory plane;
    plane.buffer_index = 0U;
    plane.plane_index = 0U;
    plane.dma_fd = 3;
    plane.mmap_length = 4096U;
    plane.fd_cloexec = true;
    plane.size_query_ok = true;
    plane.fd_size = 4096U;
    buffer.planes.push_back(plane);
    inventory.buffers.push_back(buffer);
    CHECK_TRUE(visionarm::ValidateV4L2DmabufInventory(inventory).ok);

    inventory.buffers[0].planes[0].fd_size = 1024U;
    const auto result = visionarm::ValidateV4L2DmabufInventory(inventory);
    CHECK_TRUE(!result.ok);
    CHECK_TRUE(result.error ==
               visionarm::V4L2DmabufContractError::
                   INVENTORY_FD_SIZE_TOO_SMALL);
    return true;
}

}  // namespace

int main() {
    if (!TestNv12Valid() || !TestNv12StrideFailure() ||
        !TestNv12MValid() || !TestInventory()) {
        return EXIT_FAILURE;
    }
    std::cout << "v4l2_dmabuf_contract_test PASSED\n";
    return EXIT_SUCCESS;
}
