// Temporary compatibility translation unit for the legacy CMake target.
// Final integration replaces that target with src/runtime/CMakeLists.txt and
// removes this shim; the runtime implementation is owned by src/runtime.
#include "../src/runtime/runtime_cli.cpp"
#include "../src/runtime/runtime_report.cpp"
#include "../src/runtime/runtime_app.cpp"

int main(int argc, char** argv) {
    visionarm::runtime::InstallSignalHandlers();
    return visionarm::runtime::RuntimeMain(argc, argv);
}
