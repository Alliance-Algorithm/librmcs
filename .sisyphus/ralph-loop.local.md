---
active: true
iteration: 3
max_iterations: 500
completion_promise: "DONE"
initial_completion_promise: "DONE"
started_at: "2026-04-28T21:51:45.358Z"
session_id: "ses_22b5e2171ffeu7touLmb2ZEjfO"
ultrawork: true
strategy: "continue"
message_count_at_start: 99
---
bmi088 核心目标:
**尽一切可能降低 BMI088 的陀螺仪积分零飘**.
基于这个目标, 两条并行路线:
1. 加热恒温+零飘标定
2. 压制现有链路上的丢采样 <-- 当前路线
第一版落地策略:
- pending 形态：sticky bool
- 调度策略：gyro first
- accel 处理方式：保留 accel IRQ，但降为低优先级服务
- c_board：只能在 update() 后发起下一笔
- rmcs_board：不引入 FIFO，不扩 SPI 传输层
- 这版不做统计/遥测
