# Changelog

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [Changelog](#changelog)
  * [v0.1.0 (2025-10-17)](#v010--2025-10-17-)
  * [v0.2.0 (WIP)](#v020-wip)
<!-- TOC -->

</details>

---

## [v0.1.0 (2025-10-17)](https://github.com/danielljeon/maze_bot/releases/tag/v0.1.0)

- Initial release (project Phase A).

## v0.2.0 (WIP)

- Improve `README.md` documentation.
- Swap to `TIM2` based scheduler (previously relied on DWT).
- Fix pin configurations for NUCLEO-L432KC hardware (critical).
    - Implement NUCLEO-L43KC jumper SB16 and SB18 behaviour.
        - Shuffle and remove secondary pins for each H-bridge direction pin
          pair (PA10 and PA11).
            - Implement for external signal inversion from the primary pin of
              each direction pin pair.
            - Update H-bridge drivers accordingly.
        - Move H-bridge PWM pin on PA9 (`TIM1_CH2`) to PA11 (`TIM1_CH4`).
            - Update H-bridge drivers accordingly.
        - Move VL53L4CD I2C off PB6 and PB7 to PA9 and PA 10.
- Update VL53L4CD driver for simplicity and concurrent sensor operation.
