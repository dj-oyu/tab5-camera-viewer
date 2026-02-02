#pragma once

#include <cstdint>

class DetectionData;

namespace OverlayRenderer {

// Initialize overlay rendering resources
void init();

// Render left bar (framebuffer top 160 rows, appears as left side in landscape)
// Must be called after PPA starts but before PPA completion
void renderLeftBar(uint16_t *framebuffer, DetectionData &data);

// Render right bar (framebuffer bottom 160 rows, appears as right side in landscape)
// Must be called after PPA starts but before PPA completion
void renderRightBar(uint16_t *framebuffer, DetectionData &data);

} // namespace OverlayRenderer
