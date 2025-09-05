
# `delto_gripper_helper` Library Documentation

## Overview

The **`delto_gripper_helper`** namespace provides utility functions for converting motor current to torque, enforcing current limits during torque control, and mapping target torques into PWM duty ratios.

---

## Functions

### `double GetLibraryVersion()`

* **Description**
  Returns the version number of the helper library.
  Version numbers follow a simple `1.x` scheme (e.g., `1.0`, `1.1`, `1.2`).

* **Return**
  `double` — Library version number.

---

### `std::vector<double> ConvertEffort(const std::vector<double>& current)`

* **Description**
  Converts motor current values \[mA] into output effort \[Nm].

* **Parameters**

  * `current`: Vector of motor currents \[mA].

* **Return**
  `std::vector<double>` — Vector of torque/effort values \[Nm].

---

### `std::vector<double> CurrentControl (int joint_count,  const std::vector<int>& actual_current,  const std::vector<double>& target_torque,  std::vector<int>& current_limit_flag,  std::vector<double> current_integral)
`

* **Description**
  Torque that adjusts the commanded torque to ensure
  the resulting motor current stays within safe limits.

  Internally, motor constants and PI control logic are used.
  This logic has been encapsulated into the library to hide motor-specific details.


* **Behavior**

  * If `target_torque` corresponds to current below the limit → applied directly.
  * If it would exceed current limit → torque is reduced.
  * Updates integral terms for PI control.

* **Parameters**

  * `joint_count`: Number of joints.
  * `actual_current`: Vector of measured motor currents \[mA].
  * `target_torque`: Vector of desired torques \[Nm].
  * `current_limit_flag`: Output vector; indicates current limiting per joint (`0 = normal`, `1 = limited`).
  * `current_integral`: Output vector; integral terms for each joint.

* **Return**
  `std::vector<double>` — Adjusted torque commands \[Nm].

---

### `std::vector<double> ConvertDuty(int joint_count,  std::vector<double> target_torque)`

* **Description**
  Converts desired torque \[Nm] into PWM duty cycle \[%].
  This computation uses the motor electrical model (`Kt`, `Rm`, `Vmax`).

* **Parameters**

  * `joint_count`: Number of joints.
  * `target_torque`: Vector of desired torques \[Nm].

* **Return**
  `std::vector<double>` — Duty cycle values \[%] per joint.

---


```
 ┌─────────────────┐
 │  Target Torque  │
 │      [Nm]       │
 └────────┬────────┘
          │
          │
          v
 ┌─────────────────┐     Feedback 
 │ Current Limit   │ <─────────────┐ 
 │   PI Control    │               │
 └────────┬────────┘               │
          │                        │
          v                        │
 ┌─────────────────┐               │
 │ Adjusted Torque │               │
 │      [Nm]       │               │
 └────────┬────────┘               │
          │                        │
          v                        │
 ┌─────────────────┐               │
 │  Motor Duty [%] │               │
 └────────┬────────┘               │
          │                        │
          v                        │
 ┌─────────────────┐               │
 │   Motor Driver  │───────────────┘
 └─────────────────┘

```

