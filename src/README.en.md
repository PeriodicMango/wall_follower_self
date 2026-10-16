# src/ — wall_follower version list

Naming: `wall_follower_<group>_<index>.cpp` — group 1 descends from the fallback version, group 2 from the 10-14 version.
The build always compiles `wall_follower.cpp`.

**The version that passed the assessment is `wall_follower_2_3.cpp` (passed first try).**

`START_RANGE` (radius used to decide the robot is back at the start) and `LEAVE_RANGE` (how far it must first travel away before that check is armed) are 0.20 / 1.00 in every version except `2_3`.

- **`wall_follower_1.cpp`** — original version of group 1 (fallback). Branch order R1 → R2 → R3 → R4 → R5 → R6, no `LEFT_FRONT CLOSE` branch.
  R1 turns right at −0.60; `FRONT_BLOCK_DIST` 0.50, `LEFT_OPEN_DIST` 0.60, `FRONT_LEFT_CLOSE` 0.6, `FRONT_RIGHT_CLOSE` 0.6, `LEFT_TOO_FAR_DIST` 0.50; `LINEAR_SPEED` 0.15, `TURN_LINEAR_SPEED` 0.13, `TURN_SPEED` 0.60, `GENTLE_TURN_SPEED` 0.30.

- **`wall_follower_1_1.cpp`** — same as `wall_follower_1.cpp`, only `TURN_LINEAR_SPEED` changed to 0.10.

- **`wall_follower_1_2.cpp`** — same as `wall_follower_1.cpp`, with a `LEFT_FRONT CLOSE` branch (`LEFT_FRONT < 0.30` → keep steering right) inserted after R3. 715 lines.

- **`wall_follower_2.cpp`** — group 2. Branch order R1 → R3 → `LEFT_FRONT CLOSE`(0.30) → R2 → R4 → R5 → R6.
  R1 turns right at −1.00; `FRONT_BLOCK_DIST` 0.50, `LEFT_OPEN_DIST` 0.60, `FRONT_LEFT_CLOSE` 0.3, `FRONT_RIGHT_CLOSE` 0.25, `LEFT_TOO_FAR_DIST` 0.50; `LINEAR_SPEED` 0.15, `TURN_LINEAR_SPEED` 0.13, `TURN_SPEED` 0.6, `GENTLE_TURN_SPEED` 0.30.

- **`wall_follower_2_1.cpp`** — same as `wall_follower_2.cpp`, with `TURN_LINEAR_SPEED` changed to 0.10 and the `LEFT_FRONT CLOSE` threshold changed to 0.25.

- **`wall_follower_2_2.cpp`** — same as `wall_follower_2_1.cpp`, with `FRONT_LEFT_CLOSE` / `FRONT_RIGHT_CLOSE` changed to 0.5 / 0.5 and the `LEFT_FRONT CLOSE` threshold back to 0.30.

- **`wall_follower_2_3.cpp`** — **the version that passed the assessment (first try)**. Same as `wall_follower_2_2.cpp`, plus two values:
  `START_RANGE` 0.20 → **0.30** (return-to-start radius widened to ±0.30 m), `LEAVE_RANGE` 1.00 → **0.85** (the check is armed after 0.85 m).

中文版：`README.md`
