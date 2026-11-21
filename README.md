# XYZ 加工中心开发工程

基于亚数车床／车铣源码，沿用 `CODE`、`Gdut`、`LIB_SHUKONG` 目录和原接口，逐项裁剪、修复及增补加工中心功能。

## 编译与运行

1. 用 Visual Studio 打开 `VS_Interpolation.sln`，选择 Debug 或 Release、x64，生成工程。
2. 调试工作目录设为工程根目录，命令参数留空。
3. 编辑 `resource/O0001.txt` 后运行。终端显示坐标和进给，`outputFile/test.txt` 保存周期、行号和 XYZ 指令坐标。

入口为 `main.c`。后台循环调用 `GCodeRunLoopAPI()`，`PROCESST1()` 调用 `interruptFuncAPI()`，读取程序沿用 `GCodeTest.c`。

默认 `resource/O0001.txt` 为 XYZ 基础程序，终点 X1、Y22、Z33。

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

## 代码格式

C/H 使用 UTF-8、CRLF、Microsoft／Allman 风格和 4 空格缩进。VS 编译采用 `/utf-8`。
新增源码时同步 VS 与 CMake 清单。

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
