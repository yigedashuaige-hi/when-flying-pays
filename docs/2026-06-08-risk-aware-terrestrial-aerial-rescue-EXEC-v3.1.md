# 风险感知陆空救灾机器人 —— 执行规范 (v3.1, for Codex / Claude Code)

**Risk-Aware Cross-Domain Mode Switching for Terrestrial-Aerial Rescue Robots under Traversability Uncertainty**

**Date:** 2026-06-08 · **Supersedes:** v2 (2026-06-08 design) · **Owner:** wang（1 人主导，~12 个月）

---

## PART 0 · 怎么读这份文档（重要）

这份文档有两个读者，请按角色只看对应部分：

- **Part A（给人看）** = 战略上下文：为什么这么做、创新点、和已有工作的差异。**Codex / Claude Code 不需要执行这部分**，读一遍理解意图即可。
- **Part B（给 agent 看）** = 执行规范：全局护栏 + 四个 PR + 代码级细节 + 验收标准。**这是 agent 真正要落地的部分。**
- **Part C（人类执行，禁止交给 agent）** = 实机安全流程、实验跑法、投稿与诚实声明。涉及真实硬件与人身/设备安全，**不得让 agent 自动执行**。

> 一句话主线（贯穿全文）：**地面"能不能走"是不确定的；飞行不是"更快"，而是"规避地面卡住的尾部风险"；并用一个决策层安全监督器，保证永不进入"已起飞但电量不足以安全降落"的不可逆状态。**

---

# PART A · 战略上下文（给人看，agent 略读）

## A.1 科学问题

> 在复杂救灾环境中，当机器人对地面可通行性只有不确定认知时，如何把"飞行"当作主动的风险规避动作，在地面失败风险、飞行能耗、安全落点与电量裕度之间做陆空模态决策。

## A.2 差异化（过审根基，一张表）

| 已有工作 | 做了什么 | 留给我们的空间 |
|---|---|---|
| ZJU FAST-Lab TABV 线 | 陆空**能量最优**轨迹、统一控制、真机验证、能量-时间感知探索 | 良性环境、几何/确定性可通行性；不建模**不确定/风险**，不把飞行当**风险规避** |
| JPL/MIT 越野风险线（STEP 等） | 概率可通行性 + CVaR 风险规划 | 全是**单模态地面**机器人，卡住即失败，没有"飞过去"这个动作 |
| 安全屏蔽 RL 线 | RL + CBF/可达性安全滤波（已拥挤） | 屏蔽的是**控制层避障**，非**决策层模态可行性** |

**差异化一句话：** 地面机器人不确定时只能赌，我们多一个动作——飞过去。把"用跨域能力消解地面不确定风险"形式化，是三条线的空白交集。

参考：Zhang et al., RA-L 2022 (arXiv:2109.04706)；ZJU TABV Exploration 2025 (arXiv:2507.21338)；STEP, RSS 2021；Cai et al. Probabilistic Traversability；Safety-Shielded RL Flight 2026 (arXiv:2602.08653)。

## A.3 论文核心贡献（按可辩护性排序）

1. **算法（头牌）：** 风险感知陆空模态决策——把可通行性建成分布（均值+不确定度），用 CVaR 把"地面卡住尾部风险"纳入决策。
2. **安全（最可认证）：** 决策层可达性安全监督器，机制上消除"低电起飞→不可逆坠机"，并提供 `shield_override` 实证。
3. **系统/实验：** 救灾陆空仿真系统 + **公平对照协议**（所有策略共享感知输入）+ 多场景多 seed 消融 + **单台实机低风险 sim-to-real 趋势一致性**。
4. **（第二篇）** 安全屏蔽 RL + 主动信息获取 PEEK + ROS2/PX4/Harmonic 高保真迁移。

> PEEK / "用了 RL" **不是头牌**（成熟方向，审稿人会扣分）。头牌是 1+2。

---

# PART B · Codex / Claude Code 执行规范（agent 落地这部分）

## B0 · 全局执行规则（护栏，最高优先级）

```
Codex / Claude Code Execution Rules — READ FIRST

DO:
- Work in small, reviewable PR-style steps (PR-1 → PR-4), one at a time.
- Make ADDITIVE changes only: new CSV columns, new strategies, new modules.
- Keep ALL existing strategies (ground_only / air_preferred / fixed_switch) working.
- After EACH PR, run the existing Scene A demo to confirm nothing broke.
- Put new tunable params in YAML (config/), not hard-coded.

DO NOT:
- Do NOT refactor the whole project at once.
- Do NOT migrate to ROS2 / PX4 / Gazebo Harmonic in this phase.
- Do NOT implement RL or PEEK in this phase.
- Do NOT rename or delete existing CSV columns (additive only).
- Do NOT touch unrelated launch files unless strictly necessary.
- Do NOT give risk_aware any input that energy_rule cannot see (see B1, fairness).

PROJECT: ROS1 Noetic catkin_ws, package rescue_mission (+ rescue_worlds).
Sim: Gazebo Classic 11 (set_model_state currently used for air — see B5 staging).
```

## B1 · 公平性原则（防"自我实现陷阱"，硬性验收标准）

**风险:** 若不确定性由仿真注入、又只让 `risk_aware` 使用，审稿人会判"实验偏向自己方法"。

**铁律:**
1. `scenario_sensor_sim.py` 发布的所有感知 topic，**对所有策略可见**：
   ```
   /rescue/traversability_mean    # p_trav ∈ [0,1]
   /rescue/traversability_uncertainty     # u_trav ≥ 0，可通行性认知不确定度；第一版为 proxy，非严格统计方差
   /rescue/obstacle_density
   /rescue/terrain_cost
   /rescue/visibility_confidence
   /rescue/stuck_risk_prior
   ```
2. 策略差别**只在决策逻辑**：
   - `energy_rule`：可用 `traversability_mean` + 能耗 + 距离；**禁止使用 `traversability_uncertainty`、禁止算 CVaR**。
   - `risk_aware`：在 `energy_rule` 基础上**额外**用 `traversability_uncertainty` 算风险尾部。
3. 必须能用脚本自动证明输入分布对所有策略一致（见 PR-1 的 `check_shared_inputs.py`）。

## B2 · 风险代价函数（工程可落地，正确版）

地面通过 = **两点（Bernoulli）随机结果**：以 `p_trav` 走通（名义代价 `C_nom`），以 `1 - p_trav` 卡住（巨大代价 `C_stuck`）。`u_trav`（对 `p_trav` 本身的认知不确定）用于**保守地抬高失败概率**。

```python
# ---- 共享：能量/时间名义预测，对所有策略一致 ----
E_ground_pred, T_ground_pred, E_air_pred, E_takeoff, E_land, detour, switch_penalty

# ---- energy_rule（基线，只用均值，禁用 u_trav / CVaR）----
J_ground_E = wE_g*E_ground_pred + wT_g*T_ground_pred + wR*(1 - p_trav)*C_stuck \
           + wD*detour + wS*switch_penalty
J_air_E    = wE_a*(E_takeoff + E_air_pred + E_land) + wS*switch_penalty + wL*landing_risk
mode = GROUND if J_ground_E <= J_air_E else AIR

# ---- risk_aware（本文方法，两点 CVaR + u_trav 抬升失败概率）----
# 1) 用 u_trav 保守抬高失败概率（epistemic inflation；proxy，非标定模型，见诚实声明）
p_fail_eff = clip((1 - p_trav) + k_unc * u_trav, 0.0, 1.0)   # k_unc 默认 1.0
p_succ_eff = 1 - p_fail_eff

# 2) 两点分布的 CVaR_alpha（alpha=置信度，关注最差(1-alpha)尾部；C_stuck > C_nom）
def cvar_two_point(p_succ, C_nom, C_stuck, alpha):
    tail = 1.0 - alpha
    p_fail = 1.0 - p_succ
    if p_fail >= tail:           # 整个尾部都落在"卡住"
        return C_stuck
    else:                        # 尾部 = 全部卡住质量 + 部分名义质量
        return (p_fail*C_stuck + (tail - p_fail)*C_nom) / tail

risk_cvar = cvar_two_point(p_succ_eff, C_nom=T_ground_pred, C_stuck=C_stuck, alpha=alpha)
J_ground_R = wE_g*E_ground_pred + wcvar*risk_cvar + wD*detour + wS*switch_penalty
J_air_R    = J_air_E             # 空中近似确定（安全但费电）
mode = GROUND if J_ground_R <= J_air_R else AIR
```

默认参数（写进 `config/mode_switch_params.yaml`，后做敏感性）：
```
alpha = 0.8      # CVaR 置信度
k_unc = 1.0    # 不确定度抬升系数
C_stuck = 大常数（>> 名义地面代价，体现卡住后果）
wcvar, wE_g, wT_g, wD, wS, ...  # 权重
```

机制：`u_trav`↑ ⇒ `p_fail_eff`↑ ⇒ `risk_cvar`↑ ⇒ 即使均值能耗偏好地面，`risk_aware` 也更倾向飞越。**这正是"不确定性改变决策"的可观测证据，要画出来。**

> 想更省事的 v0 起步：先用 `risk_proxy = (1-p_trav) + beta*u_trav`、`J_ground += wcvar*risk_proxy*C_stuck`，跑通后再换上面两点 CVaR。但论文主结果用两点 CVaR 版本。

每次决策记录：`J_ground, J_air, risk_cvar, energy_term, switch_reason`。

## B3 · 安全监督器规范（独立模块 `safety_supervisor.py`）

**与底层避障无关，是决策层不可逆性约束。** 输入 `proposed_action`，输出 `approved_action`。

```
Reachability-based Mode Safety Supervisor
Input: a (proposed), battery_energy, current_mode, nearest_safe_landing,
       E_air_to_land, E_return, landing_area_safety

TAKEOFF 允许  ⟺  battery_energy ≥ E_takeoff + E_air_to_safe_landing + E_land + E_margin
AIR  持续允许 ⟺  始终 ∃ 能量可达的安全落点
LAND 允许     ⟺  landing_area_safety ≥ eta_land
否则：override → GROUND / RETURN / 强制 LAND，并记录 reason
```

输出/记录：`approved_action, override_flag, override_reason, safe_landing_margin_j`，全局 `shield_override_count, unsafe_action_count, irreversible_failure`。
> `shield_override_count` 与 `irreversible_failure` 是这块贡献的**唯一硬证据**，必须落到 CSV。

## B4 · CSV Schema（仅新增列，不得改/删旧列）

```
# 已有（保留）：success, completion_time_s, total/ground/air_energy_j, switch_count,
#               path_length_m, collision_count, final_distance_m, timeout, ...
# 新增（本期）：
traversability_mean, traversability_uncertainty, obstacle_density, terrain_cost,
visibility_confidence, stuck_risk_prior,
J_ground, J_air, risk_cvar, energy_term, switch_reason,
takeoff_energy_j, land_energy_j, switch_energy_j,
stuck_event_count, slip_ratio,
shield_override_count, override_reason, safe_landing_margin_j,
unsafe_action_count, irreversible_failure
```

## B5 · set_model_state 分阶段（解决 v2 中"先 RotorS"与"先验证算法"的冲突）

```
Stage 0 (本期, PR-1~PR-4)：允许继续用 set_model_state，仅用于快速验证 risk_aware 逻辑是否成立。
Stage 1：一旦 Scene B 下 risk_aware 初步优于 energy_rule，立即接 RotorS 替换 set_model_state（air_motion_executor.py 改为发目标点，由四旋翼动力学跟踪）。
Stage 2：正式论文主实验全部使用 RotorS 结果。
```

> **铁律：`set_model_state` 仅用于算法 quick check，绝不进入最终主实验结果。** 未接 RotorS 前，论文/汇报只能写 "assuming ideal trajectory tracking"，**禁止**写 "controlled by our flight controller"。
> 注意 RotorS 只补**空中**；地面侧的打滑/卡顿物理在 PR-3 处理（否则"地面失败风险"无物理来源）。

## B6 · 四个 PR（逐个做，每个有验收 + 验证命令）

### PR-1 · 共享感知输入 + 日志列
- 改 `scenario_sensor_sim.py`：发布 B1 的 6 个 topic。
- 改 `rescue_metrics_logger.py`：记录这些输入到 CSV。
- 新增 `scripts/check_shared_inputs.py`：读取各策略 run 的 CSV，输出每个策略下 `traversability_mean / traversability_uncertainty / obstacle_density / terrain_cost / stuck_risk_prior` 的 `mean/std`；若不同策略输入分布差异超过阈值则报警（说明输入不公平，需查 seed/初始化）。
- **验收：** Scene A 旧 Demo 仍能跑；CSV 出现上述列且有值；`check_shared_inputs.py` 能输出各策略输入分布。
- **验证：** `roslaunch rescue_mission rescue_phase1_demo.launch scene:=scene_a` 跑通（`scene:=` 按实际 launch 参数名替换，务必显式指定 Scene A）；`python3 scripts/check_shared_inputs.py results/runs`。

### PR-2 · 拆 energy_rule / risk_aware
- 改 `mode_switcher.py`：保留旧策略；新增 `energy_rule`、`risk_aware`（按 B2）；两者订阅**同一份** topic；记录 `J_ground/J_air/risk_cvar/switch_reason`。
- **验收：** Scene B 下 `energy_rule` 与 `risk_aware` 各能跑 ≥5 seed 无报错；`summary.csv` 能区分两者；`check_shared_inputs.py` 显示两者输入分布一致（公平性通过）。
- **验证：** `rosrun rescue_mission run_phase1_experiments.py --scene scene_b --strategies energy_rule risk_aware --runs 5`。
- **注（命令格式）：** 若当前 `run_phase1_experiments.py` 的 `--strategies` 不支持空格多值（非 `nargs='+'`），先改成同时接受「空格或逗号分隔」两种形式；或临时用 `--strategies energy_rule,risk_aware`。别让 agent 卡在命令格式上。

### PR-3 · Scene B 主场 + 卡顿/打滑物理
- 改 `rescue_worlds`/Scene B 配置：地形含摩擦分区/松散块/低矮障碍，使地面通过**物理上真有卡住可能**（B1~B4 子场景）。
- **限制（重要）：** 优先用**参数化风险区**（摩擦区/障碍区/减速区）+ 简单 collision/friction patch + stuck 检测把效果跑出来；**不要在 PR-3 重做整个 world 几何**，改动越小越好。
- 改 `ground_goal_controller.py` 或 logger：检测并记录 `stuck_event_count`、`slip_ratio`（如低速高指令时长、速度跟踪误差）。
- **验收：** `ground_only` / `energy_rule` 在 Scene B 会产生 `stuck_event`；`risk_aware` 有机会避开（stuck 显著更少）。
- **验证：** 跑 `energy_rule` 与 `risk_aware` 各 ≥10 seed，比较 `stuck_event_count` 均值。

### PR-4 · safety_supervisor
- 新增 `safety_supervisor.py`（按 B3）；`mode_switcher` 的 `proposed_action` 必须经 supervisor 得到 `approved_action`。
- 新增 Scene D（低电量）配置；新增策略 `risk_aware_safety = risk_aware + supervisor`。
- **验收：** Scene D 下 `without supervisor` 出现 `unsafe_action`/低电起飞失败；`with supervisor` 的 `shield_override_count > 0` 且 `irreversible_failure` 相对下降。
- **验证：** 跑 `risk_aware` vs `risk_aware_safety` 于 Scene D，比较 `irreversible_failure` 与 `shield_override_count`。

## B7 · ★第一个必须通过的验证点（决定故事成立与否）

> **在 Scene B、同一份感知输入下，`risk_aware` 是否在 `stuck_event_count` / `irreversible_failure` 上显著优于 `energy_rule`？**

- 通过 → 继续 Stage 1（接 RotorS）与 PR-4 之后的实验。
- 不通过 → **先别接 RotorS、别上实机**；调 `wcvar / alpha / k_unc / C_stuck` 或加强 Scene B 的卡住风险，直到差异出现。故事不成立时投入工程是浪费。

## B8 · 投喂顺序（重要：一次只发一个 PR）

不要把整份文档丢给 agent 然后让它一口气做完 PR-1→PR-4。**逐个 PR 发，每个做完、跑通、汇报、确认后再发下一个。**

第一条可直接粘贴给 Codex / Claude Code：

```
请阅读这份 v3.1 执行规范，理解意图。现在【只执行 PR-1】：共享感知输入 topic + CSV 日志列 + check_shared_inputs.py。
严格遵守 B0 护栏：不做 PR-2/3/4，不迁 ROS2/PX4/Harmonic，不做 RL/PEEK，不重构项目，只做增量改动，不改/删旧 CSV 列。
完成后：(1) 先跑 Scene A 旧 Demo 确认旧策略仍可用；(2) 列出修改了哪些文件、CSV 新增了哪些列、如何验证；(3) 停下来等我确认，不要自行开始 PR-2。
```

PR-1 确认无误后，再用同样格式发 PR-2、PR-3、PR-4。

---

# PART C · 人类执行项（禁止交给 agent 自动执行）

## C1 · 实机安全流程（单台实机，分三级，逐级解锁）

```
Level 0：地面静态测试——只跑 safety_supervisor，禁止起飞（验证 override 逻辑）。
Level 1：离地 10–20cm，tether/软垫低空，限高限速。
Level 2：短距离低速飞行，限高、限速、急停。
```
每次实验前检查清单：`急停可用 / 限高开启 / 软垫或安全网 / 电池电压正常 / 动捕定位稳定 / 遥控随时可接管`。
> 只有一台机，炸了就停摆。**实机主打 Claim B（safety supervisor 拦截危险起飞，坠机暴露≈0）**；飞越类演示保守化。

## C2 · 实机验证 3 个 claim（小样本=趋势一致性证据，非主统计）

- **Claim B（主打，低风险）：** 软件设低电 → without vs with supervisor；指标 `unsafe_takeoff_attempt, shield_override_count, irreversible_failure, safe_landing_margin`。
- **Claim A（次，保守）：** 受控不确定地形（松散泡沫/碎石垫，物理真可能卡）；`energy_rule` vs `risk_aware`；指标 `success, stuck_event, time, switch`。
- **Claim C：** 仿真与实机**趋势一致**（不要求数值一致）。

## C3 · 诚实声明（提前写进 Limitation，避免被抓）

1. **自我实现陷阱：** 共享感知 + 仅决策逻辑不同；不确定度抬升是 proxy 而非标定 epistemic 模型（标定留 phase 2）。
2. **proxy 能耗：** 明确标注、只做相对比较、做敏感性。
3. **动机-验证缺口：** 室内 OptiTrack 干净环境 ≠ 灾区；定位为"对决策逻辑的受控验证"，不吹灾区实测。
4. **set_model_state：** 见 B5，主结果必须 RotorS。
5. **单台小样本实机：** 如实说明，主证据靠仿真。

## C4 · 投稿阶梯（人看，agent 不需要）

| 档 | 完成度 | 目标 |
|---|---|---|
| 地板 | risk_aware + supervisor + 多场景 + RotorS，实机少/无 | Sensors / Drones / IEEE Access / ROBIO / IROS Workshop / 国内 |
| 中高 | + Scene B 主场 + 多 seed + 小规模实机 | RA-L（可试投，非稳）/ IROS 正会 |
| 冲刺 | + 真实感知导出不确定性 + 实机陆空切换 + 监督器真实拦截 + PX4/Harmonic | RA-L+presentation / ICRA / TMECH / JFR |

> 一次只投一个；被拒改投下一个是常态；会议可扩成期刊；RA-L 可附 ICRA/IROS。永远有下一层接住。

---

## 最终判断

主线已收口，**不要再改主线**。本期就按 **B0 护栏 + PR-1→PR-4** 推进，第一件事是 **PR-1（共享感知 + CSV + 公平性脚本）**，紧接着跑 **B7 验证点**（Scene B 下 `risk_aware` vs `energy_rule`）。这个点过了，再接 RotorS（Stage 1）与实机；过不了，先调参不投入工程。

**计划到此为止——从现在起瓶颈是执行。**
