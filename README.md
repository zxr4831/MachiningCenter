# XYZ 加工中心开发工程

基于亚数车床／车铣源码，沿用 `CODE`、`Gdut`、`LIB_SHUKONG` 目录和原接口，逐项裁剪、修复及增补加工中心功能。

## 编译与运行

1. 用 Visual Studio 打开 `VS_Interpolation.sln`，选择 Debug 或 Release、x64，生成工程。
2. 调试工作目录设为工程根目录，命令参数留空。
3. 在 `main.c` 的 `g_cName_C0` 中选择程序编号，当前为 `0002`，读取 `resource/O0002.txt`。改为 `0001` 可复验原直槽。重新生成后运行，终端显示坐标和进给，`outputFile/test.txt` 保存周期、行号和 XYZ 指令坐标。

入口为 `main.c`。后台循环调用 `GCodeRunLoopAPI()`，`PROCESST1()` 调用 `interruptFuncAPI()`，读取程序沿用 `GCodeTest.c`。

`O0001.txt` 直槽从 X0、Y0、Z0 起步，下刀 F120、切削 F600，两层深度 Z−1／Z−2，终点 X10、Y20、Z5。配合 Φ6 平底刀，理论槽宽 6、总长 46、深 2 mm，已通过 VS 数值与 CIMCO 形状对照。

`O0002.txt` 综合验证 XY／XYZ 直线、两层圆角轮廓、G90/G91、正负 R 与非轴对齐 I/J 圆弧、半圆及整圆螺旋、G17/G18/G19，终点 X100、Y55、Z5。VS Debug／Release 的 52 段运动数值检查通过，最大几何偏差约 0.000828 mm。CIMCO 原程序与采样回放的最终形状及终点已按用户截图对照确认。使用 Φ6 平底刀、120×80×12 mm 毛坯，顶面 Z0。G18/G19 在毛坯上方验证刀路。

本机对照资料按 `outputFile/cimco/程序案例/运行日期/` 保存。每次保留输入快照 `input.nc`、坐标 `test.txt`、`reference.nc`／`replay.nc`、两份同名 `.setup` 和 `validation.json`。中间记录归入 `LocalDev`，生成资料沿用 Git 忽略规则。

以下转换命令仅用于已配置本地工具的本机，`LocalDev` 脚本未随 Git 分发。VS 重新生成并运行后执行，编号 1 用 `0001`，编号 2 用 `0002`。脚本自动归档并显示两份 NC 路径。可用 `-Executable .\x64\Debug\mc_cli.exe` 指定实际运行文件，省略时其来源记为未知。查看旧回放直接打开对应日期目录的 NC。

```powershell
powershell -ExecutionPolicy Bypass -File .\LocalDev\local-support\tools\run_cimco_case.ps1 -Program 0002
```

## 源码与预留位置

| 位置 | 用途 |
|---|---|
| `CODE/` | 译码、通道、刀补、前瞻和速度规划 |
| `Gdut/` | 运行 API、系统参数、坐标和插补 |
| `LIB_SHUKONG/` | 共享类型与公共函数 |
| `Master/` | 用户预留的模块化 SOEM 主站位置 |
| `CODE/Machine/` | 后续主站与内核衔接位置 |

当前采用单通道直角坐标 XYZ，支持 G0/G1/G2/G3、G4、G17～G21、G40、G54～G59、G90/G91、G94 和 M0/M1/M2/M30。主轴、刀长刀径补偿、换刀、孔循环及硬件联调待逐项实现。

`LocalDev/` 保存本地备份和验证资料，`.vscode/` 保存本机编辑器配置，均已忽略。历史测试、导出工具和过程说明集中在本机 `LocalDev/local-support/`。

## 格式与跳转

C/H 使用 UTF-8、CRLF、Microsoft／Allman 风格和 4 空格缩进。VS 编译采用 `/utf-8`。

`.clangd` 读取 `LocalDev/clangd/compile_commands.json`，`CMakeLists.txt` 保留相同内核源码与编译参数。本机可运行 VS Code 任务“clangd：刷新编译数据库”。换机后，在具备 MSVC、CMake 和 Ninja 的开发者终端生成数据库。

```powershell
cmake -S . -B LocalDev/clangd -G Ninja -DCMAKE_C_COMPILER=cl -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

启用 clangd 扩展后用 F12 跳转。增删源码时同步 VS 与 CMake 清单，并刷新数据库。

## 来源与对照

参考快照来自用户指定目录 `C:\Users\zxr\Desktop\newgcode\2024.9.11\VS_Interpolation`，快照日期为 2026-11-23。该目录映射到本仓库根目录，解决方案来自上一级。原作者署名和代码权属沿用原工程，早期加工中心副本的精确分叉点仍待确认。

| 标签 | 内容 |
|---|---|
| `upstream-yashu-20261121` | 亚数参考源码与项目文件 |
| `machining-center-initial` | 沿用原排版的 XYZ 加工中心初始工程 |
| `machining-center-formatted` | 统一编码和排版后的加工中心工程 |

查看从原源码到加工中心的裁剪增删。

```powershell
git diff upstream-yashu-20261121 machining-center-initial -- CODE Gdut LIB_SHUKONG main.c
```
