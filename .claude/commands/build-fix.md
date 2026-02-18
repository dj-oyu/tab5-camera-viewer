# Build and Fix

Incrementally fix build and type errors with minimal, safe changes.

## Step 1: Detect Build System

Identify the project's build tool and run the build:

| Indicator | Build Command |
|-----------|---------------|
| `platformio.ini` | `pio run 2>&1` |
| `CMakeLists.txt` | `cmake --build . 2>&1` |
| `Makefile` | `make 2>&1` |
| `Cargo.toml` | `cargo build 2>&1` |
| `package.json` with `build` script | `npm run build` or `pnpm build` |
| `go.mod` | `go build ./...` |

This is a PlatformIO project — default build command is `pio run`.

## Step 2: Parse and Group Errors

1. Run the build command and capture stderr
2. Group errors by file path
3. Sort by dependency order (fix includes/types before logic errors)
4. Count total errors for progress tracking

## Step 3: Fix Loop (One Error at a Time)

For each error:

1. **Read the file** — Use Read tool to see error context (10 lines around the error)
2. **Diagnose** — Identify root cause (missing include, wrong type, syntax error)
3. **Fix minimally** — Use Edit tool for the smallest change that resolves the error
4. **Re-run build** — Verify the error is gone and no new errors introduced
5. **Move to next** — Continue with remaining errors

## Step 4: Guardrails

Stop and ask the user if:
- A fix introduces **more errors than it resolves**
- The **same error persists after 3 attempts** (likely a deeper issue)
- The fix requires **architectural changes** (not just a build fix)
- Build errors stem from **missing dependencies** or component configuration

## Step 5: Summary

Show results:
- Errors fixed (with file paths)
- Errors remaining (if any)
- New errors introduced (should be zero)
- Suggested next steps for unresolved issues

## Recovery Strategies

| Situation | Action |
|-----------|--------|
| Missing include | Check `lib/` or `components/`; verify `#include` path |
| Type mismatch | Read both type definitions; fix the narrower type |
| Linker error | Check `platformio.ini` `lib_deps` and component paths |
| ESP-IDF SDK error | Check IDF version compatibility in `platformio.ini` |
| Clean build needed | `pio run --target clean && pio run` |

Fix one error at a time for safety. Prefer minimal diffs over refactoring.
