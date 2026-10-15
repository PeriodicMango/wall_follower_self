# src/ — wall_follower version list

Naming: `wall_follower_<group>_<index>.cpp` — group 1 descends from the fallback version, group 2 from the 10-14 version.
The build always compiles `wall_follower.cpp`.

- **`wall_follower_1.cpp`** — original version of group 1 (fallback).
  Branch order R1 → R2 → R3 → R4 → R5 → R6, no `LEFT_FRONT CLOSE` branch.
  R1 turns right at −0.60; `FRONT_BLOCK_DIST` 0.50, `LEFT_OPEN_DIST` 0.60, `FRONT_LEFT_CLOSE` 0.6, `FRONT_RIGHT_CLOSE` 0.6, `LEFT_TOO_FAR_DIST` 0.50; `LINEAR_SPEED` 0.15, `TURN_LINEAR_SPEED` 0.13, `TURN_SPEED` 0.60, `GENTLE_TURN_SPEED` 0.30.

- **`wall_follower_1_1.cpp`** — same as `wall_follower_1.cpp`, only `TURN_LINEAR_SPEED` changed to 0.10.

- **`wall_follower_2.cpp`** — group 2.
  Branch order R1 → R3 → `LEFT_FRONT CLOSE`(0.25) → R2 → R4 → R5 → R6.
  R1 turns right at −1.00; `FRONT_BLOCK_DIST` 0.50, `LEFT_OPEN_DIST` 0.60, `FRONT_LEFT_CLOSE` 0.3, `FRONT_RIGHT_CLOSE` 0.25, `LEFT_TOO_FAR_DIST` 0.50; `LINEAR_SPEED` 0.15, `TURN_LINEAR_SPEED` 0.13, `TURN_SPEED` 0.6, `GENTLE_TURN_SPEED` 0.30.

- **`wall_follower_2_1.cpp`** — same as `wall_follower_2.cpp`, only `TURN_LINEAR_SPEED` changed to 0.10.

- **`wall_follower_2_2.cpp`** — same as `wall_follower_2_1.cpp`, only `FRONT_LEFT_CLOSE` and `FRONT_RIGHT_CLOSE` changed to 0.5.

中文版：`README.md`
