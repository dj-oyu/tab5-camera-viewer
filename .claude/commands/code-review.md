# Code Review

Comprehensive security and quality review of uncommitted changes:

1. Get changed files: git diff --name-only HEAD

2. For each changed file, check for:

**Security Issues (CRITICAL):**
- Hardcoded credentials, WiFi passwords, API keys, auth tokens
- Buffer overflows or out-of-bounds access
- Use of unsafe functions (strcpy, sprintf without bounds)
- Missing input validation on external data (HTTP responses, SSE streams)

**Code Quality (HIGH):**
- Functions > 50 lines
- Files > 800 lines
- Nesting depth > 4 levels
- Missing error handling (unchecked return values, unhandled queue failures)
- Magic numbers without named constants
- TODO/FIXME comments

**C++/Embedded Best Practices (MEDIUM):**
- Raw `new`/`delete` instead of RAII or smart pointers
- Missing `const` on variables/parameters that don't change
- Plain `enum` instead of `enum class`
- `NULL` or `0` used as pointer (use `nullptr`)
- Memory leaks (especially in SPIRAM allocations)
- FreeRTOS task handles or semaphores not properly cleaned up
- Missing `noexcept` on functions that must not throw

3. Generate report with:
   - Severity: CRITICAL, HIGH, MEDIUM, LOW
   - File location and line numbers
   - Issue description
   - Suggested fix

4. Block commit if CRITICAL or HIGH issues found

Never approve code with security vulnerabilities or memory safety issues!
