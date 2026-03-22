# 仓库指南

## 项目结构与模块组织
- `core/`：共享协议、数据模型和工具代码（`core/include/librmcs`、`core/src`）。
- `host/`：桌面端 SDK/库源码和公共头文件（`host/include/librmcs`、`host/src`），构建产物为 `librmcs-sdk`。
- `firmware/rmcs_board/`：基于 HPM 的板卡固件。
- `firmware/c_board/`：STM32 固件，分为 `app/` 和 `bootloader/` 两部分。
- `firmware/*/bsp/`：厂商/子模块依赖；除非有意更新子模块，否则视为第三方代码。
- `.scripts/`：与 CI 对齐的工具脚本（`clang-format-check`、`clang-tidy-check`、`generate_version`）。

## 构建、测试与开发命令
```bash
cmake --preset linux-debug -S host
cmake --build host/build

cmake --preset debug -S firmware/rmcs_board
cmake --build firmware/rmcs_board/build

cmake --preset debug -S firmware/c_board
cmake --build firmware/c_board/build --target c_board_app c_board_bootloader
```

```bash
.scripts/clang-format-check --fix    # 应用格式修复
.scripts/clang-tidy-check            # 静态分析（需在 CMake configure 之后运行）
```

`.scripts/clang-tidy-check --fix` 可触发 clang-tidy 自动修复，但部分修复可能不符合预期，需手动调整。
例如: int var 会被修复为 int const var. 但项目中使用的 Google 风格要求使用 const int var。

## 代码风格与命名规范
- 语言：C11 + C++23，禁用 GNU 扩展。
- 格式：4 空格缩进，100 列宽度限制，指针左对齐，启用 include 排序。
- 命名：Google 风格，但函数命名为小写下划线。
- 代码不允许包含任何非 ASCII 字符（Markdown 文档除外）。

## 测试指南
- 目前尚未启用 CTest/GTest 测试目标；当前 CI 质量门禁为：clang-format、clang-tidy 和 编译验证。
- 每次修改后，应在本地运行 lint 工具，并至少构建一个相关的构建目标。

## 提交指南
- Git unstaged changes 是必要的 code review 渠道。Agent 严禁执行 git add。 
- Commit Message 全英文，不允许包含任何非 ASCII 字符。
- 遵循仓库的 Conventional Commit 风格；破坏性变更使用 `!` 标记。
- 冒号后首字母需大写：`feat(scope): Capitalize the first letter of the title`。
- 每次提交应聚焦于单个模块或关注点。