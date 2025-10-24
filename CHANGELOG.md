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
    - NUCLEO-L43KC jumper SB16 and SB18 behaviour.
        - Secondary pins for each H-bridge direction pair (PA10 and PA11)
          shuffled and removed.
            - Signal inversion handled externally from the primary pin of each
              pair.
            - H-bridge drivers updated accordingly.
        - H-bridge PWM pin on PA9 moved to PA11 (`TIM1_CH4`).
        - VL53L4CD I2C moved off PB6 and PB7 to PA9 and PA 10.
