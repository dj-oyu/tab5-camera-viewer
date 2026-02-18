---
name: cpp-coding-standards
description: C++ coding standards based on the C++ Core Guidelines (isocpp.github.io). Use when writing, reviewing, or refactoring C++ code to enforce modern, safe, and idiomatic practices.
---

# C++ Coding Standards (C++ Core Guidelines)

Comprehensive coding standards for modern C++ (C++17/20/23) derived from the [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines). Enforces type safety, resource safety, immutability, and clarity.

## Cross-Cutting Principles

1. **RAII everywhere** (P.8, R.1, E.6, CP.20): Bind resource lifetime to object lifetime
2. **Immutability by default** (P.10, Con.1-5, ES.25): Start with `const`/`constexpr`; mutability is the exception
3. **Type safety** (P.4, I.4, ES.46-49, Enum.3): Use the type system to prevent errors at compile time
4. **Express intent** (P.3, F.1, NL.1-2, T.10): Names, types, and concepts should communicate purpose
5. **Minimize complexity** (F.2-3, ES.5, Per.4-5): Simple code is correct code
6. **Value semantics over pointer semantics** (C.10, R.3-5, F.20, CP.31): Prefer returning by value and scoped objects

## Functions (F.*)

```cpp
// F.16: Cheap types by value, others by const&
void print(int x);                           // cheap: by value
void analyze(const std::string& data);       // expensive: by const&

// F.20 + F.21: Return values, not output parameters
struct ParseResult { std::string token; int position; };
ParseResult parse(std::string_view input);   // GOOD

// F.4 + F.8: Pure, constexpr where possible
constexpr int factorial(int n) noexcept {
    return (n <= 1) ? 1 : n * factorial(n - 1);
}
```

## Classes (C.*)

```cpp
// C.20: Rule of Zero — let compiler generate special members
struct FrameMetadata {
    uint32_t timestamp;
    size_t size;
    // No destructor needed
};

// C.21: Rule of Five — if you manage a resource, define all five
class LinearBuffer {
public:
    explicit LinearBuffer(size_t size)
        : data_(static_cast<uint8_t*>(heap_caps_malloc(size, MALLOC_CAP_SPIRAM)))
        , size_(size) {}
    ~LinearBuffer() { heap_caps_free(data_); }
    LinearBuffer(const LinearBuffer&) = delete;
    LinearBuffer& operator=(const LinearBuffer&) = delete;
    LinearBuffer(LinearBuffer&& o) noexcept : data_(o.data_), size_(o.size_) { o.data_ = nullptr; }
    LinearBuffer& operator=(LinearBuffer&&) noexcept = default;
private:
    uint8_t* data_;
    size_t size_;
};

// C.46: Explicit single-argument constructors
explicit Buffer(std::size_t size);  // prevents implicit conversion

// C.128: Use override
class MyTask : public BaseTask {
    void run() override;  // not virtual run()
};
```

## Resource Management (R.*)

```cpp
// R.3: Raw pointer = non-owning observer
void render(const uint8_t* buf) { /* does NOT own buf */ }

// R.11: No naked new/delete — use RAII or smart pointers
auto widget = std::make_unique<Widget>();  // R.20 + R.21

// For embedded SPIRAM: RAII wrapper for heap_caps_malloc
class SpiramBuffer {
public:
    explicit SpiramBuffer(size_t n)
        : ptr_(static_cast<uint8_t*>(
              heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT))) {}
    ~SpiramBuffer() { heap_caps_free(ptr_); }
    uint8_t* get() { return ptr_; }
private:
    uint8_t* ptr_;
};
```

## Constants & Immutability (Con.*)

```cpp
// Con.1-5: Immutability by default
const uint32_t frame_width = 960;       // Con.4
constexpr int MAX_QUEUE_DEPTH = 3;      // Con.5
void process(const FrameMetadata& m);   // Con.3
```

## Concurrency (CP.*) — FreeRTOS context

```cpp
// CP.20 + CP.44: RAII locks, always named
void push(int value) {
    std::lock_guard<std::mutex> lock(mutex_);  // named!
    queue_.push(value);
}

// CP.42: Always wait with condition
cv_.wait(lock, [this] { return !queue_.empty(); });

// CP.8: volatile is NOT for synchronization — use std::atomic or FreeRTOS primitives
std::atomic<bool> task_running{true};   // GOOD
volatile bool task_running = true;      // BAD for synchronization
```

## Error Handling (E.*)

```cpp
// E.14 + E.15: Custom exception types, throw by value, catch by reference
class PipelineError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

// E.6: Use RAII to prevent leaks — never naked resource management
// E.12: noexcept when appropriate (FreeRTOS callbacks, destructors)
~MyTask() noexcept { vTaskDelete(task_handle_); }
```

## Expressions & Statements (ES.*)

```cpp
// ES.20 + ES.23: Always initialize, prefer {} initializer
const int max_retries{3};
uint8_t* buf{nullptr};

// ES.47: nullptr not NULL or 0
if (ptr != nullptr) { ... }

// ES.45: No magic numbers
constexpr size_t JPEG_BUFFER_SIZE = 82 * 1024;  // from PipelineConfig.h

// ES.48: No C-style casts
auto* typed = static_cast<uint8_t*>(raw_ptr);  // GOOD
auto* typed = (uint8_t*)raw_ptr;               // BAD
```

## Enumerations (Enum.*)

```cpp
// Enum.3: enum class, not plain enum
enum class TaskState { idle, running, error };
enum class DisplayRotation { portrait, landscape };

// BAD:
enum { IDLE, RUNNING, ERROR };  // leaks names, ALL_CAPS conflicts with macros
```

## Headers (SF.*)

```cpp
// SF.8: #pragma once (preferred over include guards in this project)
#pragma once

// SF.7: Never using namespace in headers
// SF.11: Headers must be self-contained — include everything they need
```

## Quick Reference Checklist

Before marking C++ work complete:

- [ ] No raw `new`/`delete` — use smart pointers or RAII (R.11)
- [ ] Objects initialized at declaration (ES.20)
- [ ] Variables are `const`/`constexpr` by default (Con.1, ES.25)
- [ ] Member functions are `const` where possible (Con.2)
- [ ] `enum class` instead of plain `enum` (Enum.3)
- [ ] `nullptr` instead of `0`/`NULL` (ES.47)
- [ ] No C-style casts — use `static_cast` etc. (ES.48)
- [ ] Single-argument constructors are `explicit` (C.46)
- [ ] Rule of Zero or Rule of Five applied (C.20, C.21)
- [ ] `override` on all virtual function overrides (C.128)
- [ ] Locks use RAII, named guards (CP.20, CP.44)
- [ ] `'\n'` instead of `std::endl` (SL.io.50)
- [ ] No magic numbers — use `PipelineConfig.h` constants (ES.45)
- [ ] `#pragma once` in all new headers (SF.8)
- [ ] No `using namespace` in headers (SF.7)
