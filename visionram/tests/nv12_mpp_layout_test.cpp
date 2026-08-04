#include "video/nv12_mpp_layout.h"

#include <cstdlib>
#include <iostream>
#include <linux/videodev2.h>

namespace {

void Expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

visionarm::CaptureFrameView MakeFrame() {
    visionarm::CaptureFrameView frame;
    frame.width = 1280;
    frame.height = 720;
    frame.pixel_format = V4L2_PIX_FMT_NV12;
    frame.plane_count = 1U;
    frame.planes[0].mapped_address = reinterpret_cast<void*>(0x1000);
    frame.planes[0].dma_fd = 3;
    frame.planes[0].stride = 1280U;
    frame.planes[0].size_image = 1280U * 720U * 3U / 2U;
    frame.planes[0].allocation_length = frame.planes[0].size_image;
    frame.planes[0].bytes_used = frame.planes[0].size_image;
    return frame;
}

}  // namespace

int main() {
    auto frame = MakeFrame();
    visionarm::Nv12MppLayout layout;
    Expect(
        visionarm::DeriveNv12MppLayout(frame, 0, &layout),
        "derive tightly packed NV12 layout");
    Expect(layout.horizontal_stride == 1280, "horizontal stride");
    Expect(layout.vertical_stride == 720, "vertical stride");

    frame.planes[0].stride = 1344U;
    frame.planes[0].size_image = 1344U * 736U * 3U / 2U;
    frame.planes[0].allocation_length = frame.planes[0].size_image;
    Expect(
        visionarm::DeriveNv12MppLayout(frame, 736, &layout),
        "accept explicit aligned vertical stride");
    Expect(layout.vertical_stride == 736, "aligned vertical stride");

    frame.pixel_format = V4L2_PIX_FMT_NV12M;
    Expect(
        !visionarm::DeriveNv12MppLayout(frame, 736, &layout),
        "reject NV12M");

    frame = MakeFrame();
    frame.planes[0].data_offset = 128U;
    Expect(
        !visionarm::DeriveNv12MppLayout(frame, 720, &layout),
        "reject non-zero data offset");

    frame = MakeFrame();
    frame.planes[0].allocation_length -= 1U;
    Expect(
        !visionarm::DeriveNv12MppLayout(frame, 720, &layout),
        "reject undersized allocation");

    frame = MakeFrame();
    frame.planes[0].size_image = 1280U * 721U;
    frame.planes[0].allocation_length = frame.planes[0].size_image;
    Expect(
        !visionarm::DeriveNv12MppLayout(frame, 0, &layout),
        "reject non-integral derived vertical stride");

    std::cout << "nv12_mpp_layout_test PASSED\n";
    return EXIT_SUCCESS;
}
