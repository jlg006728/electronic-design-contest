# 当前烧录工程说明

本目录是从当前实际使用的 CCS 工程复制而来：

```text
C:\Users\1\Desktop\ELECTRIC\ele-car
```

复制时间：2026-07-02 21:15。

未复制 `Debug`、`Release` 等编译产物。核心文件：

- `empty.c`：当前实际烧录的循迹程序。
- `empty.syscfg`：SysConfig 配置。
- `.ccsproject`、`.cproject`、`.project`：CCS 工程文件。
- `targetConfigs/MSPM0G3507.ccxml`：目标板调试配置。

如果 CCS 导入该工程失败，使用更稳妥的方式：

1. 新建 MSPM0G3507 empty project。
2. 用 `ccs_project/line_sensor_test/20260702_210925_line_follow_wide_turn_lost_timeout.c` 替换新工程的 `empty.c`。
3. 保持之前的 `empty.syscfg` 或重新按 MSPM0G3507 默认 empty project 生成。

