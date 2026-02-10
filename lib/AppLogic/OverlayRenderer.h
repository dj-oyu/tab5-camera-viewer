#pragma once

#include <cstdint>

class BatteryData;
class DetectionData;
class ConnectionData;
class RecordingData;

namespace OverlayRenderer {

// Initialize sprite and static resources (called once)
void init();

// Physical layout (landscape, USB left, camera right):
// - Framebuffer top (rows 0-159) → Camera side (right)
// - Framebuffer bottom (rows 1120-1279) → USB side (left)

// Render both overlay bars
void render(DetectionData &detectionData, ConnectionData &connectionData,
            RecordingData &recordingData, BatteryData &batteryData,
            uint16_t *framebuffer, float render_fps = -1.0f);

} // namespace OverlayRenderer
