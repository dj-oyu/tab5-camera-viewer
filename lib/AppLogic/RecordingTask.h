#pragma once

#include <freertos/FreeRTOS.h>

class PipelineContext;

namespace RecordingTask {

void start(PipelineContext &ctx, UBaseType_t priority, BaseType_t core);

} // namespace RecordingTask
