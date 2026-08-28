#include "runtime/runtime_app.h"

int main(int argc, char** argv) {
    visionarm::runtime::InstallSignalHandlers();
    return visionarm::runtime::RuntimeMain(argc, argv);
}
