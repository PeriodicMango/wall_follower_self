# src/ — wall_follower 版本列表

命名 `wall_follower_<版本组>_<序号>.cpp`：版本组 1 = 保底那一支；版本组 2 = 10-14 那支。
编译的永远是 `wall_follower.cpp`。

**评审通过的是 `wall_follower_2_3.cpp`（一次过）。**

`START_RANGE`（回到起点的判定半径）与 `LEAVE_RANGE`（先离开多远才允许判定回到起点），除 `2_3` 外都是 0.20 / 1.00。

- **`wall_follower_1.cpp`** —— 版本组 1 的原始版本（保底）。分支顺序 R1 → R2 → R3 → R4 → R5 → R6，没有 `LEFT_FRONT CLOSE` 分支。
  R1 右转 −0.60；`FRONT_BLOCK_DIST` 0.50、`LEFT_OPEN_DIST` 0.60、`FRONT_LEFT_CLOSE` 0.6、`FRONT_RIGHT_CLOSE` 0.6、`LEFT_TOO_FAR_DIST` 0.50；`LINEAR_SPEED` 0.15、`TURN_LINEAR_SPEED` 0.13、`TURN_SPEED` 0.60、`GENTLE_TURN_SPEED` 0.30。

- **`wall_follower_1_1.cpp`** —— 同 `wall_follower_1.cpp`，只把 `TURN_LINEAR_SPEED` 改成 0.10。

- **`wall_follower_1_2.cpp`** —— 同 `wall_follower_1.cpp`，在 R3 之后插入 `LEFT_FRONT CLOSE` 分支（`LEFT_FRONT < 0.30` → 继续向右修）。715 行。

- **`wall_follower_2.cpp`** —— 版本组 2。分支顺序 R1 → R3 → `LEFT_FRONT CLOSE`(0.30) → R2 → R4 → R5 → R6。
  R1 右转 −1.00；`FRONT_BLOCK_DIST` 0.50、`LEFT_OPEN_DIST` 0.60、`FRONT_LEFT_CLOSE` 0.3、`FRONT_RIGHT_CLOSE` 0.25、`LEFT_TOO_FAR_DIST` 0.50；`LINEAR_SPEED` 0.15、`TURN_LINEAR_SPEED` 0.13、`TURN_SPEED` 0.6、`GENTLE_TURN_SPEED` 0.30。

- **`wall_follower_2_1.cpp`** —— 同 `wall_follower_2.cpp`，把 `TURN_LINEAR_SPEED` 改成 0.10、`LEFT_FRONT CLOSE` 阈值改成 0.25。

- **`wall_follower_2_2.cpp`** —— 同 `wall_follower_2_1.cpp`，把 `FRONT_LEFT_CLOSE` / `FRONT_RIGHT_CLOSE` 改成 0.5 / 0.5、`LEFT_FRONT CLOSE` 阈值改回 0.30。

- **`wall_follower_2_3.cpp`** —— **评审通过的那一版（一次过）**。同 `wall_follower_2_2.cpp`，只多两处：
  `START_RANGE` 0.20 → **0.30**（回到起点的判定半径放宽到 ±0.30 m）、`LEAVE_RANGE` 1.00 → **0.85**（离开 0.85 m 就解除门控）。

English: `README.en.md`
