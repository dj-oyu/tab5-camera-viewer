# Verification Command

Run comprehensive verification on current codebase state.

## Instructions

Execute verification in this exact order:

1. **Build Check**
   - Run `pio run 2>&1`
   - If it fails, report errors and STOP

2. **Static Analysis** (if available)
   - Check for obvious issues: buffer overflows, uninitialized variables
   - Report all warnings with file:line

3. **Memory Audit**
   - Check `PipelineConfig.h` buffer sizes vs SPIRAM available (16MB)
   - Verify no allocation exceeds physical limits
   - Check for potential stack overflows in FreeRTOS tasks

4. **Git Status**
   - Show uncommitted changes
   - Show files modified since last commit

5. **Debug Output Check**
   - Search for `ESP_LOGD` / `printf` / `Serial.print` left in production paths
   - Report locations

## Output

Produce a concise verification report:

```
VERIFICATION: [PASS/FAIL]

Build:    [OK/FAIL]
Memory:   [OK/WARN - describe issue]
Warnings: [OK/X issues]
Git:      [clean/X uncommitted files]
Debug:    [OK/X debug prints found]

Ready for PR: [YES/NO]
```

If any critical issues, list them with fix suggestions.

## Arguments

$ARGUMENTS can be:
- `quick` - Only build check
- `full` - All checks (default)
- `pre-commit` - Build + git status + debug check
- `pre-pr` - Full checks
