# Copilot Instructions for Embedded C++20 Project
This project is a modern C++20 project for the **NUCLEO-H753ZI** evaluation board, which features an **STM32H753ZI** microcontroller.

## Architecture & Constraints
* Target MCU: STM32H753ZI (Cortex-M7, 400MHz, 1MB Flash, 512KB RAM).
* Language: C++20 (G++ 14+).
* Build System: CMake 3.20+ & Ninja.
* Toolchain: `arm-none-eabi-gcc`.
* Debug/Flash: ST-LINK (SWD) via OpenOCD.
* Platform: Manjaro Linux.

## Coding Guidelines
* Language Standard: Strictly C++20
* Heavily utilize `constexpr` and `consteval` for compile-time computations to save flash/RAM
* Use C++20 `requires` clauses and `concepts` for template metaprogramming instead of SFINAE
* Use `std::span` for array passing.
* Correctness over cleverness
* Clarity over brevity
* Deterministic behavior
* Explicit ownership and lifetimes

## Performance Rules
* Avoid heap allocations in hot paths.
* Use std::string_view for non-owning string parameters.
* Use reserve() when size is known.
* Prefer emplace_back.
* Avoid exceptions in tight loops.
* Prefer contiguous memory layouts.

# Output Requirements for AI
* Output compile-ready C++20.
* Include only required includes.
* Avoid full file rewrites unless requested.
* Keep diffs minimal.

# Dependency Rules
* Only use approved dependencies.
* Never introduce new third-party libraries without review.
* Prefer the C++ standard library over external solutions.

# Scripting
* Use Python 3.10+ for any build scripts or utilities.
