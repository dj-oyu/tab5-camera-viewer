# verify-sdkconfig.ps1 — Verify custom_sdkconfig settings in sdkconfig.defaults
#
# Parses platformio.ini custom_sdkconfig section and checks that each setting
# is present in the generated sdkconfig.defaults after build.

$sdkconfigFile = "sdkconfig.defaults"
$iniFile = "platformio.ini"

if (!(Test-Path $sdkconfigFile)) {
    Write-Host "ERROR: $sdkconfigFile not found. Run 'make build' first." -ForegroundColor Red
    exit 1
}

if (!(Test-Path $iniFile)) {
    Write-Host "ERROR: $iniFile not found." -ForegroundColor Red
    exit 1
}

# Parse custom_sdkconfig from platformio.ini
$settings = @()
$inSection = $false
foreach ($line in Get-Content $iniFile) {
    if ($line -match '^\s*custom_sdkconfig\s*=') {
        $inSection = $true
        $after = ($line -split '=', 2)[1].Trim()
        if ($after) { $settings += $after }
        continue
    }
    if ($inSection) {
        if ($line -match '^\s+\S') {
            $settings += $line.Trim()
        } else {
            break
        }
    }
}

if ($settings.Count -eq 0) {
    Write-Host "No custom_sdkconfig settings found in $iniFile" -ForegroundColor Yellow
    exit 0
}

Write-Host "Checking $($settings.Count) settings from custom_sdkconfig:" -ForegroundColor White

# Verify each setting in sdkconfig.defaults
$allOk = $true
foreach ($setting in $settings) {
    # Skip comments and empty lines
    if (!$setting -or $setting.StartsWith(';') -or $setting.StartsWith('#')) { continue }

    # Skip =n settings (Kconfig disable flags may not appear in output)
    if ($setting -match '=n$') {
        Write-Host "  --  $setting" -ForegroundColor DarkGray
        continue
    }

    $match = Select-String -Path $sdkconfigFile -Pattern $setting -SimpleMatch
    if ($match) {
        Write-Host "  OK  $setting" -ForegroundColor Green
    } else {
        Write-Host "  NG  $setting" -ForegroundColor Red
        $allOk = $false
    }
}

if ($allOk) {
    Write-Host "`nAll custom_sdkconfig settings verified." -ForegroundColor Green
} else {
    Write-Host "`nSome settings are missing or different." -ForegroundColor Yellow
    Write-Host "Try: make clean-build" -ForegroundColor Yellow
    exit 1
}
