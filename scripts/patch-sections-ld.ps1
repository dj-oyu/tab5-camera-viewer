# patch-sections-ld.ps1 — Apply orphan section catch-all fix to sections.ld
#
# pioarduino v54.03.21's prebuilt sections.ld is missing a catch-all for
# .text.* sections from prebuilt .a libraries (e.g. libhal.a inline functions).
# Orphan sections create extra ELF segments exceeding esptool's max 16 limit.
#
# Fix: Insert *(.text .text.*) after *(.irom0.text) in .flash.text section.

$sectionsLd = Join-Path $env:USERPROFILE `
    ".platformio/packages/framework-arduinoespressif32-libs/esp32p4/ld/sections.ld"

if (!(Test-Path $sectionsLd)) {
    Write-Host "ERROR: sections.ld not found:" -ForegroundColor Red
    Write-Host "  $sectionsLd" -ForegroundColor Red
    Write-Host "Run 'pio run' once to install the framework." -ForegroundColor Yellow
    exit 1
}

# Check if already patched
if (Select-String -Path $sectionsLd -Pattern 'Catch-all.*orphan' -Quiet) {
    Write-Host "sections.ld: already patched" -ForegroundColor Cyan
    exit 0
}

# Read and patch
$content = Get-Content $sectionsLd -Raw

$patchBlock = @"

    /* Catch-all: absorb orphan .text.* sections from prebuilt libs
       whose function names are missing from the explicit listings above.
       Without this, orphan sections create extra ELF segments (max 16). */
    *(.text .text.*)
"@

$patched = $content -replace '(\*\(\.irom0\.text\)[^\r\n]*)', "`$1$patchBlock"

if ($patched -eq $content) {
    Write-Host "ERROR: Could not find '*(.irom0.text)' anchor in sections.ld" -ForegroundColor Red
    Write-Host "  The file format may have changed. Manual patching required." -ForegroundColor Yellow
    exit 1
}

Set-Content -Path $sectionsLd -Value $patched -NoNewline
Write-Host "sections.ld: patched successfully" -ForegroundColor Green
Write-Host "  $sectionsLd" -ForegroundColor DarkGray
