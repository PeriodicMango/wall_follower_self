# src/ — wall_follower 版本列表

命名规则 `wall_follower_<版本组>_<序号>.cpp`：版本组 1 = 保底那一支；版本组 2 = 10-14 那支。
编译的永远是 `wall_follower.cpp`。

- **`wall_follower_1.cpp`** —— 版本组 1 的原始版本（保底）。
  分支顺序 R1 → R2 → R3 → R4 → R5 → R6，没有 `LEFT_FRONT CLOSE` 分支。
  R1 右转 −0.60；`FRONT_BLOCK_DIST` 0.50、`LEFT_OPEN_DIST` 0.60、`FRONT_LEFT_CLOSE` 0.6、`FRONT_RIGHT_CLOSE` 0.6、`LEFT_TOO_FAR_DIST` 0.50；`LINEAR_SPEED` 0.15、`TURN_LINEAR_SPEED` 0.13、`TURN_SPEED` 0.60、`GENTLE_TURN_SPEED` 0.30。

- **`wall_follower_1_1.cpp`** —— 同 `wall_follower_1.cpp`，只把 `TURN_LINEAR_SPEED` 改成 0.10。

- **`wall_follower_2.cpp`** —— 版本组 2。
  分支顺序 R1 → R3 → `LEFT_FRONT CLOSE`(0.25) → R2 → R4 → R5 → R6。
  R1 右转 −1.00；`FRONT_BLOCK_DIST` 0.50、`LEFT_OPEN_DIST` 0.60、`FRONT_LEFT_CLOSE` 0.3、`FRONT_RIGHT_CLOSE` 0.25、`LEFT_TOO_FAR_DIST` 0.50；`LINEAR_SPEED` 0.15、`TURN_LINEAR_SPEED` 0.13、`TURN_SPEED` 0.6、`GENTLE_TURN_SPEED` 0.30。

- **`wall_follower_2_1.cpp`** —— 同 `wall_follower_2.cpp`，只把 `TURN_LINEAR_SPEED` 改成 0.10。

- **`wall_follower_2_2.cpp`** —— 同 `wall_follower_2_1.cpp`，只把 `FRONT_LEFT_CLOSE` 和 `FRONT_RIGHT_CLOSE` 改成 0.5。

English: `README.en.md`
