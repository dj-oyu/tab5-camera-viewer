#pragma once

#include <cstdint>

class DetectionData;
class ConnectionData;

namespace OverlayRenderer {

// Initialize with framebuffer pointer (called once)
void init(uint16_t *framebuffer);

// Physical layout (landscape, USB left, camera right):
// - Framebuffer top (rows 0-159) → Camera side (right)
// - Framebuffer bottom (rows 1120-1279) → USB side (left)

// Render both overlay bars
void render(DetectionData &detectionData, ConnectionData &connectionData);

} // namespace OverlayRenderer
