# 加工中心 H743 主站

SOEM 主站模块同步自独立台架仓库 `stm32-soem-cubeide` 的 `eb1a00334e4a70e99b01a0ee7182d8746f1209d3`。
整机由 `Core/Src/main.c` 初始化数控与轴保护参数，`Core/Src/tim.c` 在 1 ms 周期调用主站与插补。
