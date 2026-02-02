# ISA Flight Software: MISRA C Compliance Audit Report

**Date:** 2026-02-02  
**Scope:** `include/` and `src/` directories  
**Standard:** MISRA C:2012 (Guidelines for the Use of the C Language in Critical Systems)

## 1. Executive Summary
The ISA Flight Software codebase currently exhibits a high degree of modularity and follows several fundamental safety-critical programming practices (e.g., restricted use of dynamic memory). However, the code **partially satisfies** the MISRA C:2012 standard. Full compliance requires a refactoring pass to address type consistency, mathematical error handling, and dead code removal.

---

## 2. Identified Rule Violations

### 🔴 High Priority (Required Rules)

#### Rule 2.2: Dead Code
- **Finding:** In `src/minor.c`, the `check_sensor_health()` function declares variables (e.g., `accel_x_ok`, `gyro_x_ok`) that are calculated but never consumed by subsequent logic.
- **Risk:** Indicates potential logic gaps where sensor health is identified but not acted upon.
- **Recommendation:** Integrate health flags into the `SystemState` or remove unused local calculations.

#### Rule 8.4: External Linkage
- **Finding:** The global object `systemState` is defined in `major.c` and declared `extern` in `minor.c`, but it is not consistently declared in a shared header that both files include.
- **Risk:** Linker errors or type mismatches if structure definitions diverge.
- **Recommendation:** Ensure all shared global variables are declared in `include/system_state.h` and included in every `.c` file using them.

#### Rule 21.1: Standard Library Security
- **Finding:** Direct use of `<math.h>` functions (`acos`, `sqrt`) is common without consistent input range validation.
- **Risk:** Passing invalid values (e.g., `acos(1.0001)`) triggers undefined behavior or `NaN`.
- **Recommendation:** Mandate the use of the `safe_*` wrappers already present in `math_utils.c` for all mathematical operations.

### 🟡 Medium Priority (Advisory Rules)

#### Rule 4.6: Basic Type Usage
- **Finding:** Widespread use of basic C types like `int` and `double` in `src/minor.c` and `src/major.c`.
- **Risk:** Non-portable behavior across different platform architectures and word sizes.
- **Recommendation:** Standardize on `<stdint.h>` types (e.g., `int32_t`, `uint16_t`) and a project-defined `float64_t`.

#### Rule 12.1: Implicit Operator Precedence
- **Finding:** Complex mathematical expressions in `src/math_utils.c` and `src/major.c` rely on default C precedence.
- **Risk:** Visual ambiguity and potential programmer error during maintenance.
- **Recommendation:** Use explicit parentheses (e.g., `(a * b) + (c * d)` instead of `a * b + c * d`).

---

## 3. Notable Compliant Areas
The codebase successfully adheres to several critical MISRA rules:

| Rule | Status | Note |
| :--- | :--- | :--- |
| **Rule 15.6 (Required)** | ✅ Compliant | All `if`, `else`, and `for` blocks correctly use braces `{ }`. |
| **Rule 21.3 (Required)** | ✅ Compliant | No usage of dynamic memory allocation (`malloc`, `free`). |
| **Rule 10.1 (Required)** | ✅ Compliant | Bitwise operations consistently use the `U` suffix for unsigned types. |
| **Rule 17.1 (Required)** | ✅ Compliant | Recursive function calls were not identified in the core flight loop. |

---

## 4. Path to Certification
To achieve full "MISRA Compliant" status, the following roadmap is recommended:
1.  **Type Refactoring:** Perform a global find-and-replace to substitute native types with fixed-width equivalents.
2.  **Dead Variable Cleanup:** Remove or utilize all flags identified in `minor.c`.
3.  **Math Sanitization:** Audit all `math_utils.c` functions to ensure they meet explicit precedence and safe-range requirements.
4.  **Static Analysis:** Integrate a professional MISRA C static analyzer (e.g., PC-lint, Polyspace, or C-Report) into the build pipeline.

---
