cmake_minimum_required(VERSION 3.21)
project(MachiningCenterXYZ VERSION 0.1.0 LANGUAGES C)
# 当前构建目标为 Windows 桌面数控内核，H743 板级支持包和 SOEM 按独立模块接入。
if(NOT WIN32)
  message(FATAL_ERROR "This baseline uses the inherited Windows host IO facade.")
endif()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_library(mc_kernel STATIC
  "CODE/Channel/Channel.c"
  "CODE/CTrajPlanning/CTrajPlanning.c"
  "CODE/Decod&Check/command/GCodeG0.c"
  "CODE/Decod&Check/command/GCodeG1.c"
  "CODE/Decod&Check/command/GCodeG171819.c"
  "CODE/Decod&Check/command/GCodeG2.c"
  "CODE/Decod&Check/command/GCodeG20G21.c"
  "CODE/Decod&Check/command/GCodeG3.c"
  "CODE/Decod&Check/command/GCodeG4.c"
  "CODE/Decod&Check/command/GCodeG404142.c"
  "CODE/Decod&Check/command/GCodeG54.c"
  "CODE/Decod&Check/command/GCodeG90G91.c"
  "CODE/Decod&Check/command/GCodeGoto.c"
  "CODE/Decod&Check/command/GCodeIfAndWhile.c"
  "CODE/Decod&Check/command/MCodeMb.c"
  "CODE/Decod&Check/GCodeAssign.c"
  "CODE/Decod&Check/GCodeCalcExpression.c"
  "CODE/Decod&Check/GCodeCheck.c"
  "CODE/Corner/CornerTransition.c"
  "CODE/Corner/CornerRoundAdapter.c"
  "CODE/Decod&Check/GCodeCheckAndAssign.c"
  "CODE/Decod&Check/GCodeCommandAPI.c"
  "CODE/Decod&Check/GCodeDefine.c"
  "CODE/Decod&Check/GCodeMain.c"
  "CODE/Decod&Check/GCodeProgram.c"
  "CODE/Decod&Check/GCodeQuickHandle.c"
  "CODE/Decod&Check/GCodeRegulation.c"
  "CODE/Decod&Check/GCodeTest.c"
  "CODE/Decod&Check/getAxisMaxAndMin.c"
  "CODE/ErrorCompensation/ErrorCompensation.c"
  "CODE/Handwheel/Handwheel.c"
  "CODE/Handwheel/HandwheelSim.c"
  "CODE/IOCommand/IOCommand.c"
  "CODE/list/list.c"
  "CODE/lookAhead/lookAhead.c"
  "CODE/manageMem/manageMem.c"
  "CODE/Motion/Motion.c"
  "CODE/RingBuff/ringbuff.c"
  "CODE/SecondSpindle/SecondSpindle.c"
  "CODE/Spindle/Spindle.c"
  "CODE/TIOCommand/TIOCommand.c"
  "CODE/toolAxis/toolAxis.c"
  "CODE/ToolCompensation/ToolCompensation.c"
  "CODE/tree/tree.c"
  "Gdut/API/coordAPI.c"
  "Gdut/API/motionAPI.c"
  "Gdut/API/RunAPI.c"
  "Gdut/API/SysParaAPI.c"
  "Gdut/API/ToolChanger.c"
  "Gdut/G0/G0.c"
  "Gdut/G1&G2G3/G1.c"
  "Gdut/G1&G2G3/G2.c"
  "Gdut/G32/G32.c"
  "Gdut/G33/G33.c"
  "LIB_SHUKONG/GdutLib/common.c"
  "LIB_SHUKONG/GdutLib/ErrorFunction.c"
  "LIB_SHUKONG/GdutLib/GeneralFunctions.c"
)
file(GLOB_RECURSE MC_HEADERS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/CODE/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/Gdut/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/LIB_SHUKONG/*.h"
)
set(MC_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/Gdut/API")
foreach(header IN LISTS MC_HEADERS)
  get_filename_component(header_dir "${header}" DIRECTORY)
  list(APPEND MC_INCLUDE_DIRS "${header_dir}")
endforeach()
list(REMOVE_DUPLICATES MC_INCLUDE_DIRS)
target_include_directories(mc_kernel PUBLIC ${MC_INCLUDE_DIRS})
target_compile_definitions(mc_kernel PUBLIC STM32_MODE=0 _CRT_SECURE_NO_WARNINGS _CRT_NONSTDC_NO_DEPRECATE)
if(MSVC)
  target_compile_options(mc_kernel PRIVATE /W3)
endif()
add_executable(mc_cli main.c)
target_link_libraries(mc_cli PRIVATE mc_kernel)
if(MSVC)
  target_compile_options(mc_cli PRIVATE /W4)
endif()
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT mc_cli)
