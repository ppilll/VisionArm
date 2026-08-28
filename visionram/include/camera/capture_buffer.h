#pragma once

// Stable aggregate for the capture broker, frame-lease contract, and
// requeue completion API. The previous two headers remain supported during
// the staged refactor so non-capture owners need no include-path changes.
#include "camera/capture_buffer_contract.h"
#include "camera/capture_buffer_broker.h"
