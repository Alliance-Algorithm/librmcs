# Planning: rmcs_board USB 无超时 Bug 修复与 bootloader 看门狗

> 状态：已实施（见文末"实施记录"，git 提交与 submodule push 待人工执行）
> 范围：T1 hpm_usb_drv IRQ 轮询核心修复、T2 rmcs_board bootloader 看门狗、T3 c_board bootloader 看门狗、T4 c_board tinyusb DWC2 同类修复
> 范围外（暂缓）：Virtual 板开发、SDK 线程态无超时轮询全面治理

---

## 1. 背景与问题

### 1.1 故障现象

lite（以及 pro，共用同一 BSP）板在物理震动后出现 USB 失联/系统异常。审查定位到
`usb_dcd_bus_reset()` 在 **USB 中断上下文**中存在无超时忙等：

```c
// firmware/rmcs_board/bsp/hpm_sdk/drivers/src/hpm_usb_drv.c:96
void usb_dcd_bus_reset(USB_Type *ptr, uint16_t ep0_max_packet_size)
{
    ...
    while (ptr->ENDPTPRIME) {        // :117 无超时
    }
    ptr->ENDPTFLUSH = 0xFFFFFFFF;    // :119
    while (ptr->ENDPTFLUSH) {        // :120 无超时
    }
}
```

### 1.2 完整调用链（均在 IRQ 上下文）

```
USB 总线复位/PHY 异常 → PLIC IRQn_USB0
  → isr_usb0                              dcd_hpm.c:327-328
    → dcd_int_handler                     dcd_hpm.c:225
      → (int_status & intr_reset)         dcd_hpm.c:244-245
        → bus_reset()                     dcd_hpm.c:90-92
          → usb_device_bus_reset()        hpm_usb_device.c:72-76
            → usb_dcd_bus_reset()         hpm_usb_drv.c:96-122
              → while (ENDPTPRIME) {}     hpm_usb_drv.c:117   <-- 挂死点
              → while (ENDPTFLUSH) {}     hpm_usb_drv.c:120   <-- 挂死点
```

`dcd_event_bus_reset()`（`dcd_hpm.c:247`）在 `bus_reset()` 返回后才会入队；循环挂死时复位事件永远
无法送达 `tud_task()`。

### 1.3 根因

ChipIdea UDC（HPM USB 控制器）的 `ENDPTPRIME`/`ENDPTFLUSH` 是硬件握手位，依赖端点队列与时钟域
正常运转。物理震动导致 PHY/控制器/端点状态异常后，这些位可能永不清零。HPM SDK 全栈
（`hpm_usb_drv.c`、`hpm_usb_drv.h`、`hpm_usb_device.c`、`dcd_hpm.c`）**没有任何一处轮询带超时**，
也无错误恢复路径——即"被 hpm 驱动阴了"：驱动假设硬件永远正常。

### 1.4 故障后果分级

| 场景 | 现状后果 | 保护 |
| --- | --- | --- |
| app 运行中挂死 | 主循环饿死（`tud_task`、UART/CAN/GPIO、feed 全停），约 500ms 后 EWDG 复位，表现为 USB 反复掉线复位循环 | EWDG 500ms（`app/src/watchdog/watchdog.hpp:38`，仅从主循环 feed，`app.cpp:63`） |
| bootloader DFU 中挂死 | **永久卡死直到断电**（bootloader 无任何看门狗） | 无 |
| `board_init`/`board_init_usb` 阶段挂死 | app 看门狗尚未启动（`app.cpp:27-28` < `:32`），永久卡死 | 无 |
| IRQ 影响范围 | HPM ISR 包装默认开嵌套（`hpm_interrupt.h` `ENTER_NESTED_IRQ_HANDLING_M`），同级及低优先级 IRQ 被阻塞，thread mode 完全饿死 | — |

### 1.5 c_board 同类问题

c_board（STM32F4 + tinyusb DWC2 port）存在完全同类的 IRQ 态无超时轮询，见 T4。

---

## 2. 无超时轮询全量清单（SDK）

### 2.1 IRQ 上下文（本次修复重点）

| # | file:line | 内容 | 归属 |
| --- | --- | --- | --- |
| 1 | `hpm_usb_drv.c:117-118` | `while (ENDPTPRIME)`（bus reset） | T1 |
| 2 | `hpm_usb_drv.c:120-121` | `while (ENDPTFLUSH)`（bus reset） | T1 |
| 3 | `dcd_hpm.c:280` | `while (1)` 遍历 qTD 链（ENDPTCOMPLETE 处理，链损坏时死循环） | T1 评估 |
| 4 | `dcd_dwc2.c:306/310/328/332` | `edpt_disable` 系列等待（SETUP 清理等从 IRQ 到达） | T4 |
| 5 | `dwc2_common.h:97/103` | TX/RX FIFO flush 等待（bus reset 路径 `dcd_dwc2.c:761-762`） | T4 |
| 6 | `dcd_dwc2.c:1186-1188` | RXFLVL 排空 `do {} while` | T4 |

### 2.2 线程上下文（记录在案，暂不处理）

| file:line | 内容 | 备注 |
| --- | --- | --- |
| `hpm_usb_drv.c:54-56` | `usb_phy_deinit` UTMI reset 状态 | init 路径，MIE 关闭 |
| `hpm_usb_drv.c:83-85` | `usb_phy_init` UTMI_CLK_VALID | 同上 |
| `hpm_usb_drv.c:141-142` / `:183-184` | `usb_dcd_init/deinit` USBCMD_RST | 同上 |
| `hpm_usb_drv.c:215-216` | `usb_dcd_disconnect` SOF 等待 | app 未调用 |
| `hpm_usb_drv.c:289-301` | `usb_dcd_edpt_close` flush + ENDPTSTAT | tud_task 中 |
| `hpm_usb_device.c:277-279` | EP0 setup lockout 等待 | tud_task 中 |
| `hpm_usb_device.c:262-265` / `:290-293` | `while (1)` 断言式挂死 | 故意 |
| `boards/{pro,lite}/board.c` sysctl busy 等待 | `board_init` 内 | 看门狗启动前窗口 |

---

## 3. 修复位置选型（先分析后定）

背景事实：

- `hpm_sdk` 是 **git submodule**：`Alliance-Algorithm/hpm_sdk`（本组织 fork，有 push 权限），
  当前 pinned `a82a5354`（`dev/rmcsboard-bootloader` 分支），工作区干净。
  另有 `upstream = hpmicro/hpm_sdk`（fetch only）。相关文件与 fork `origin/main` 无差异。
- 所有调用点（`usb_dcd_bus_reset` ← `usb_device_bus_reset` ← `dcd_hpm.c`）**全部位于 submodule 内部**。
- SDK 通过 `drivers/CMakeLists.txt:25` `sdk_src_ifdef(... src/hpm_usb_drv.c)` 编入 `HPM_SDK_LIB` 静态库。
- SDK 内不存在任何超时版本实现（全仓 grep 无），需自写。

### 方案 A：直接改 submodule fork（推荐）

流程：在 fork 上基于 `dev/rmcsboard-bootloader` 提交修复 → push → 主仓库 bump gitlink。

- 优点：改动最小、语义清晰；同一 SDK 的 cherryusb/ThreadX 端口也受益；无构建系统 hack。
- 风险：gitlink bump 前其他 clone 拉不到该 commit（须先 push fork）；fork 与 upstream 后续合并可能冲突
  （冲突面小：仅 USB 驱动几处循环）。
- 符合仓库现状：该 fork 本就是为 rmcs_board 定制维护的（`dev/rmcsboard-bootloader` 分支名即证据）。

### 方案 B：主仓库 overlay/wrapper

在 librmcs 内提供替换符号，链接时覆盖 SDK 静态库版本。

- 否决理由：`usb_dcd_bus_reset` 等被 `hpm_usb_device.c`（同库）直接调用，静态库内符号无法被
  外部定义覆盖；需要重编整个 SDK 对象集，等价于 hack 构建系统，比方案 A 更侵入。

### 方案 C：configure 期打补丁

CMake configure 时对 submodule 工作树自动 `git apply` 补丁。

- 保留 submodule "干净"表象，但：CI/本地可重复性依赖补丁与 pinned 版本严格同步；submodule 工作树被
  改脏会干扰日常 `git status`/切分支；补丁漂移是长期负担。
- 仅当"禁止改 fork"时才考虑。

### 结论

**采用方案 A**（T1 改 `Alliance-Algorithm/hpm_sdk`、T4 改 `qzhhhi/tinyusb` fork，均先 push 再 bump
gitlink）。若组织层面禁止直接改 fork，降级为方案 C，实施时再决定（保留 A/C 两套描述）。

---

## 4. 详细修改计划

### T1（核心）hpm_usb_drv.c IRQ 轮询超时 + 错误传播

**改动文件（全部在 `firmware/rmcs_board/bsp/hpm_sdk` submodule 内）：**

1. `drivers/src/hpm_usb_drv.c`
   - `usb_dcd_bus_reset()`：两处 `while` 改为**有界等待**。
   - 新增统一的有界等待辅助（函数形式，遵循 SDK 现有风格；如 SDK 已有宏风格则跟随）。
   - 超时后**不再死等**：记录失败并继续执行后续清理（ENDPTFLUSH 已写入，硬件最终态交给恢复流程）。
2. `drivers/inc/hpm_usb_drv.h` / `components/usb/device/hpm_usb_device.c`
   - `usb_dcd_bus_reset` → `usb_device_bus_reset` 增加失败返回（`hpm_stat_t` 或 `bool`，跟随 SDK
     既有返回风格——SDK 内多用 `hpm_stat_t`）。
3. `middleware/tinyusb/src/portable/hpm/dcd_hpm.c`
   - `bus_reset()`（:90）与 `dcd_int_handler`（:244-247）：失败时**不入队** `dcd_event_bus_reset`，
     改为进入恢复路径（见下）。

**超时数值标定：**

- 优先用轮询次数上界（无依赖、IRQ 内安全）：按 480MHz CPU、每次迭代 ~3-5 cycle 估算，
  目标上界 **1~2ms**（枚举正常路径 prime/flush 完成在微秒级，留 >100 倍余量）。
- 备选：读 `HPM_MCHTMR` mtime 绝对时间（`timer.hpp:47-64` 已有防撕裂读法可参考），更直观但引入
  时钟依赖（bootloader/早期 init 阶段 mchtmr 时钟组未必开启，`board.c:96` 才加入）——
  **默认用迭代计数**，常量命名 `kXxxTimeoutLoops` 并注释推算依据。
- 实施时以实测枚举耗时复核（高速枚举、连续 100 次插拔无误报超时）。

**超时后恢复策略：**

1. `usb_dcd_bus_reset` 超时 → 返回失败。
2. `dcd_int_handler` 收到失败：屏蔽 USB 中断（`usb_disable_interrupts` 或写 `USBINTR`），
   置全局恢复标志（`dcd_hpm.c` 内 `static volatile bool` + 查询接口，或经 `usb_device_*` 暴露）。
3. 恢复动作（二选一，实施时按验证结果定）：
   - **a. 软恢复**：主循环检测标志 → 调 `tusb_deinit`/`tusb_rhport_init` 全量重初始化
     （`usb_dcd_init` 自身有 USBCMD_RST 无超时等待，属线程态风险，见 2.2——若线程态不修，
     软恢复仍可能在重初始化时挂死，由 EWDG 兜底）。
   - **b. 硬兜底**：仅屏蔽中断，依靠 EWDG 500ms 复位（等价现状但 IRQ 不再长期占死、
     同级 IRQ 不再被压制，缩短故障窗口）。
   - 推荐先实现 b（改动小、确定性高），a 作为增强项在验证阶段评估。
3. `dcd_hpm.c:280` 的 `while (1)` qTD 遍历：加链长/next 合法性上界（同 commit 顺带，改动小）。

**明确不做（本任务）**：2.2 表中线程态轮询——列为后续任务，避免本次 diff 过大；但 T1 的软恢复方案
a 依赖其中之一，届时联动。

### T2 rmcs_board bootloader 加看门狗

**现状：** `bootloader/src/main.cpp` 主循环（`tud_task` + `Dfu::poll`）无任何看门狗；DFU 期间 USB
IRQ 挂死 = 永久卡死。

**方案：**

1. 新增 `firmware/rmcs_board/bootloader/src/watchdog/watchdog.hpp`（或直接复用
   `app/src/watchdog/watchdog.hpp`——该头自包含（仅依赖 `lazy.hpp` 与 HPM 驱动），bootloader 已
   `sdk_app_inc(project root)`（`bootloader/CMakeLists.txt:72`），可直接
   `#include "firmware/rmcs_board/app/src/watchdog/watchdog.hpp"`。**优先直接复用，不新建文件**）。
2. `main()` 中在 `board_init()` **之后**初始化（`clock_add_to_group(clock_watchdog0, 0)` 需要时钟组
   配置，`board.c` 的时钟初始化先完成）；跳 app 路径（`jump_to_app`）不启动看门狗，避免影响 app 侧。
3. 主循环每圈 `feed()`。

**关键约束（擦除 vs 超时竞态）：**

- `XpiNor::erase_sector`（`bootloader/src/flash/xpi_nor.hpp:31-42`）在擦除期间
  **`disable_global_irq`（:34）**，期间无法 feed；W25Q 4KB sector erase 最坏规格约 400ms。
- 500ms 超时余量过薄。**方案：擦除/编程前显式 `feed()`，并将 bootloader 超时放宽到 1s**
  （复用 app 头则需给 `kTimeoutUs` 参数化/构造参数；若参数化改动大，独立复制一份 bootloader
  版 watchdog 头也可接受——实施时二选一，倾向参数化）。
- 兜底：若擦除实测 > 超时，调大超时而非加 feed 循环（擦除期间 IRQ 关闭，无法插 feed）。

**注意：** `metadata.hpp` 的 `erase_and_rescan()`（:156-157）也会走同一擦除路径，需覆盖。

### T3 c_board bootloader 加看门狗

**现状：** `firmware/c_board/bootloader/src/main.cpp` 同样无看门狗（app 侧 IWDG 见
`app/src/watchdog/watchdog.hpp`，0.5s）。

**方案：**

1. 复用 `app/src/watchdog/watchdog.hpp`（IWDG，含 `DBGMCU` freeze 配置；bootloader include 路径
   已含 project root——`c_board/bootloader/CMakeLists.txt:40`）。
2. `main()` 中 `HAL_Init`/`SystemClock_Config` 后初始化；跳 app 不启动（app 自己会启动；
   **IWDG 一旦启动不可停止**，若 bootloader 启动后跳 app，app 的 watchdog.init() 会重新配置同一
   IWDG——需确认重复 init 安全（LL_IWDG 写访问重复配置是安全的），或跳 app 前不启动）。
3. 主循环 feed。

**关键约束：**

- 大扇区擦除：app 区 `SECTOR_7/11` 为 128KB，擦除最坏**秒级**（STM32F4 规格）；
  `writer.hpp:148`、`metadata.hpp:193` 的 `HAL_FLASHEx_Erase` 阻塞执行期间 IWDG 持续计数
  （IWDG 硬件独立于 CPU）。
- **方案：擦除前 feed + bootloader 超时设为覆盖最坏擦除时长（预估 2~4s，按 DS 手册
  tSE(max) 标定）**，或擦除循环内分批擦（改动大，不推荐）。
- 与 T2 一样：超时宁长勿短，DFU 误复位比挂死更糟（中断的 flash 写入有 metadata/校验兜底，
  但会挫败用户）。

### T4 c_board tinyusb DWC2 全部 IRQ 上下文循环

**改动文件（`firmware/c_board/bsp/tinyusb` submodule，fork `qzhhhi/tinyusb`，分支 `v0.20.0`）：**

| file:line | 内容 | 上下文 |
| --- | --- | --- |
| `src/portable/synopsys/dwc2/dcd_dwc2.c:306` | `while ((dep->diepint & INEPNE)==0)` | IRQ（SETUP 清理）+ 线程 |
| `dcd_dwc2.c:310` | `while ((dep->diepint & EPDISD)==0)` | 同上 |
| `dcd_dwc2.c:328` | `while ((gintsts & BOUTNAKEFF)==0)`（上游注释自认风险） | 同上 |
| `dcd_dwc2.c:332` | `while ((dep->doepint & EPDISD)==0)` | 同上 |
| `dwc2_common.h:97/103` | TX/RX FIFO flush 等待 | IRQ（bus reset `dcd_dwc2.c:761-762`）+ 线程 |
| `dcd_dwc2.c:1186-1188` | RXFLVL `do {} while` 排空 | IRQ |

**实施要点：**

- 先核对 upstream `hathach/tinyusb` 新版本是否已有修复（本机网络受限时用本地 `upstream` remote
  已 fetch 的引用；`git log`/`git diff origin/v0.20.0..upstream/master -- src/portable/synopsys/dwc2/`），
  有则回移，无则自写有界等待（与 T1 同一标定思路）。
- 与 T1 保持一致的超时语义：超时不阻塞 IRQ 返回，失败路径交由上层/IWDG 兜底。
- c_board app 已有 IWDG 0.5s 兜底（`app.cpp:41` 启动，USB init 在 `:62` 之后，窗口安全）。

---

## 5. 验证计划

### 5.1 构建与静态检查

```bash
# 编译矩阵（四目标）
cmake --preset linux-debug -S host && cmake --build host/build
cmake --preset debug -S firmware/rmcs_board -DBOARD=pro  && cmake --build firmware/rmcs_board/build
cmake --preset debug -S firmware/rmcs_board -DBOARD=lite && cmake --build firmware/rmcs_board/build
cmake --preset debug -S firmware/c_board && cmake --build firmware/c_board/build \
    --target c_board_app c_board_bootloader

.scripts/clang-format-check --fix
.scripts/clang-tidy-check
```

- 提交前 grep 复查：修改后的 `hpm_usb_drv.c` / `dcd_dwc2.c` 中不再存在裸 `while (...) {}`
  等待硬件位（IRQ 相关行号全部消除）。

### 5.2 硬件在环（lite/pro + c_board）

| # | 项目 | 通过标准 |
| --- | --- | --- |
| 1 | 正常枚举回归 | 100 次插拔/复位风暴全部正常枚举，无误报超时日志 |
| 2 | DFU 长刷写 | 大镜像连续刷写 10 次，看门狗不误复位（T2/T3 核心验收） |
| 3 | bootloader 挂死注入 | 调试器在 `usb_dcd_bus_reset` 循环内模拟卡死 → 看门狗复位可恢复（验证 T2/T3 兜底） |
| 4 | app 挂死注入 | 同上注入 app 模式 → EWDG 500ms 复位；确认 USB IRQ 挂死期间其余 IRQ 行为改善 |
| 5 | 超时误报压测 | 高速枚举 + 低速 hub + 反复热插拔，确认 1~2ms 上界不误触发 |
| 6 | 抗震动 | 复现原故障工况（震动/扰动 USB），确认不再出现永久失联 |

---

## 6. 提交切分（Conventional Commit 草案）

依赖顺序：**先 push submodule fork，再 bump 主仓库 gitlink**，否则其他 clone/CI 断链。

| 序 | 仓库 | 草案 |
| --- | --- | --- |
| 1 | `Alliance-Algorithm/hpm_sdk` | `fix(usb): Add bounded waits to device DCD prime and flush loops` |
| 2 | librmcs（gitlink） | `fix(rmcs_board)!: Bound USB IRQ busy-waits via hpm_sdk bump` |
| 3 | librmcs | `feat(rmcs_board): Add watchdog to bootloader` |
| 4 | `qzhhhi/tinyusb` | `fix(dwc2): Bound IRQ-context endpoint and FIFO flush waits` |
| 5 | librmcs（gitlink + 代码） | `fix(c_board): Bound USB IRQ busy-waits and add bootloader watchdog` |

- T2/T3 若拆开则各一条 `feat(...): Add watchdog to bootloader`。
- 主仓库提交聚焦单模块（submodule bump 与功能代码不混同一 commit，除非 gitlink bump 本身即该功能）。
- Agent 不执行 `git add`；diff 留在工作区供 review。

---

## 7. 风险与开放问题

1. **submodule 先 push 后 bump 的协作风险**：本地 submodule commit 未 push 前，其他 clone 构建即失败。
   实施顺序必须严格按第 6 节。
2. **超时误判风险**：过短会掐断正常慢速枚举。1~2ms 上界初值 + 5.2-#5 压测复核。
3. **擦除 vs 看门狗竞态数据**：W25Q sector erase / STM32 128KB sector erase 的最坏时长需按各自
   datasheet 确认（预估 400ms / 2~4s），T2/T3 超时以实测 + 规格双保险。
4. **软恢复（T1-a）与线程态无超时轮询的耦合**：`usb_dcd_init` 的 USBCMD_RST 等待若不修，软恢复
   重初始化仍可能挂死（由 EWDG 兜底）。软恢复列为验证期增强项，不阻塞主修复。
5. **fork 与 upstream 漂移**：两处 fork 修复日后合并 upstream 可能冲突；冲突面小，可接受。
6. **范围外待办**：SDK 线程态轮询治理（2.2 表）、`board_init` 看门狗启动前窗口、
   Virtual 板开发（暂缓，另行规划）。

---

## 8. 实施记录（2026-09-28）

### 改动清单

**T1 — hpm_sdk submodule（`firmware/rmcs_board/bsp/hpm_sdk`，需先 push fork 再 bump gitlink）：**

- `drivers/inc/hpm_usb_drv.h`：新增 `USB_DCD_HW_CLEAR_WAIT_LIMIT`（100000 次迭代，约 1-4ms @360-480MHz）；
  `usb_dcd_bus_reset` 返回 `hpm_stat_t`。
- `drivers/src/hpm_usb_drv.c`：新增 `usb_dcd_wait_reg_clear()` 有界等待助手；
  `usb_dcd_bus_reset` 的 `ENDPTPRIME`/`ENDPTFLUSH` 轮询加超时，超时返回 `status_timeout`。
- `components/usb/device/hpm_usb_device.c|h`：`usb_device_bus_reset` 传播失败状态；失败时屏蔽
  `USBINTR` 并置恢复标志；新增 `usb_device_recovery_required()` 查询接口。
- `middleware/tinyusb/src/portable/hpm/dcd_hpm.c`：`bus_reset()` 返回 bool，失败时不入队
  `dcd_event_bus_reset` 并提前返回；ENDPTCOMPLETE 的 qTD 链遍历加 NULL 检查与
  `USB_SOC_DCD_QTD_COUNT_EACH_ENDPOINT` 长度上界。

**主仓库（rmcs_board）：**

- `app/src/app.cpp`：主循环检查 `usb_device_recovery_required()`，置位时停止 feed（EWDG 复位）；
  以 `extern "C"` 声明代替包含 `hpm_usb_device.h`（该头 C 位域无法过 clang-tidy）。
- `app/src/watchdog/watchdog.hpp`：超时经 `LIBRMCS_WATCHDOG_TIMEOUT_US` 参数化（默认 500ms）。
- `bootloader/src/main.cpp`：跳转决策后尽早 `watchdog.init()`（覆盖 `board_init_usb`/`tusb_rhport_init`
  的无超时等待）；主循环 feed + 恢复标志检查；超时经 CMake 定义放宽为 **1s**。
- `bootloader/CMakeLists.txt`：`LIBRMCS_WATCHDOG_TIMEOUT_US=1000000`。
- `bootloader/src/flash/xpi_nor.hpp`：`erase_sector` 在关全局 IRQ 前 `feed()`（W25Q 4KB 擦除最坏
  ~400ms，1s 预算覆盖擦除+编程）。
- `bootloader/src/utility/assert.cpp`（新增）：bootloader 专用 `core::utility::assert_func`
  （app 版依赖 ws2812，bootloader 不可用）。

**T3 — c_board 主仓库：**

- `app/src/watchdog/watchdog.hpp`：reload 经 `LIBRMCS_WATCHDOG_RELOAD` 参数化（默认 250 = 0.5s）。
- `bootloader/CMakeLists.txt`：`LIBRMCS_WATCHDOG_RELOAD=2000`（**4s**，覆盖 128KB 扇区最坏擦除+编程）。
- `bootloader/src/main.cpp`：跳转决策后 `watchdog.init()`；主循环 feed + 恢复标志检查。
- `bootloader/src/flash/writer.hpp`、`metadata.hpp`：擦除前 `feed()`。
- `bootloader/src/utility/assert.cpp`（新增）：同 T2 理由。

**T4 — tinyusb submodule（`firmware/c_board/bsp/tinyusb`，`v0.20.0` 分支，需先 push 再 bump）：**

- `src/portable/synopsys/dwc2/dwc2_common.h`：新增 `DWC2_HW_WAIT_LIMIT`（1000000，约 90ms @168MHz）、
  `DWC2_RXFLVL_DRAIN_LIMIT`（65536）、`dwc2_hw_report_timeout()`/`dwc2_recovery_required()` 声明；
  `dfifo_flush_tx/rx` 改为有界（bus reset 路径的 FIFO flush）。
- `src/portable/synopsys/dwc2/dwc2_common.c`：超时上报实现——`gintmsk = 0` 屏蔽全部核心中断
  + 置恢复标志。
- `src/portable/synopsys/dwc2/dcd_dwc2.c`：`edpt_disable` 的 INEPNE/EPDISD/BOUTNAKEFF/EPDISD
  四处循环有界化（超时上报并提前返回）；RXFLVL 排空循环加包数上界。
- `app/src/app.cpp`、`bootloader/src/main.cpp`：检查 `dwc2_recovery_required()`，置位停止 feed。

**clang-tidy 适配（主仓库）：**

- 两个 `watchdog.hpp` 补直接 include（`hpm_common.h` / 无需）。
- `hpm_usb_device.h` 的 C 匿名位域在 C++ 下为 clang 诊断错误——app/bootloader 改为 `extern "C"`
  声明，不再包含该头。
- 新增 assert.cpp 的 `main.h` 加 `// IWYU pragma: keep`。

### 验证结果

| 检查 | 结果 |
| --- | --- |
| `rmcs_board` pro + lite（app+bootloader，CI 容器） | 构建通过 |
| `c_board`（app+bootloader，CI 容器） | 构建通过 |
| `host` SDK 构建 | 通过（无改动） |
| `.scripts/clang-format-check` | 通过（147 files） |
| `.scripts/clang-tidy-check`（host + rmcs_board + c_board） | 通过（TIDY_EXIT=0） |
| grep 复查：HPM bus reset / DWC2 IRQ 轮询 | 已全部有界 |

### 待人工执行

1. 按第 6 节顺序：两个 submodule 内提交 → push 对应 fork → 主仓库提交（gitlink bump + 仓库代码）。
2. 硬件在环验证（第 5.2 节 6 项），重点：DFU 长刷写不误复位、挂死注入可恢复、正常枚举无超时误报。
3. 超时数值按实测微调（`USB_DCD_HW_CLEAR_WAIT_LIMIT` / `DWC2_HW_WAIT_LIMIT`）。
