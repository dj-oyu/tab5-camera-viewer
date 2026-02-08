# Makefile — Tab5 Camera Viewer (Hybrid Compile workflow)
#
# Prerequisites: GNU Make, PlatformIO CLI
#   Install make: choco install make  /  scoop install make
#
# Usage:
#   make              Build firmware
#   make upload       Build & flash to device
#   make monitor      Serial monitor (115200 bps)
#   make clean-build  Full clean + sections.ld patch + build (~5 min)
#   make clean        Remove build artifacts
#   make patch-ld     Apply sections.ld orphan section fix
#   make verify       Verify sdkconfig.defaults settings
#
# Note: If build fails with "Invalid segment count (max 16)", run:
#   make patch-ld
#   make build
#
# Note: If CMake configuration error causes framework reinstall,
#   sections.ld patch may be overwritten. Re-run: make patch-ld

.DEFAULT_GOAL := build
.PHONY: build upload monitor clean clean-build patch-ld verify help

# --- Build targets ---

build:
	pio run

upload:
	pio run --target upload

monitor:
	pio run --target monitor

# --- Hybrid Compile workflow ---

clean:
	@if exist ".pio\build" rmdir /s /q ".pio\build"
	@if exist ".dummy" rmdir /s /q ".dummy"
	@if exist "managed_components" rmdir /s /q "managed_components"
	@if exist "sdkconfig.defaults" del "sdkconfig.defaults"
	@echo Clean complete.

patch-ld:
	@powershell -NoProfile -ExecutionPolicy Bypass -File scripts\patch-sections-ld.ps1

clean-build: clean patch-ld build

verify:
	@powershell -NoProfile -ExecutionPolicy Bypass -File scripts\verify-sdkconfig.ps1

# --- Help ---

help:
	@echo.
	@echo   make              Build firmware (default)
	@echo   make upload       Build and flash to device
	@echo   make monitor      Serial monitor (115200 bps)
	@echo   make clean-build  Full clean + patch + build (~5 min)
	@echo   make clean        Remove build artifacts
	@echo   make patch-ld     Apply sections.ld orphan section fix
	@echo   make verify       Verify sdkconfig.defaults after build
	@echo.
