within;
package UAVSafetyChain "姿态安全链：参考可行性整形 + 倾角指令限速(ERG) + 姿态CBF"
  extends Modelica.Icons.Package;

  annotation (Documentation(info = "<html>
<h4>设计依据</h4>
<p>对 03_Simulation_Results 中六组真实仿真输出的复算表明：全部 6 个 15&deg; 姿态超限点
（climb_enhanced 3 个、spiral_baseline 1 个、spiral_enhanced 2 个）100% 落在参考轨迹
不连续时刻之后的 1 秒窗口内，220 s 飞行的其余时段无一超限。</p>
<p>触发条件：参考不连续 &rarr; 位置环 PID 微分项被激励（Derivative 滤波常数 T=0.01 与
采样步长同量级，微分增益约 100）&rarr; 倾角指令在 3~4 个采样点内打满限幅。</p>
<p><b>但参考不连续只是必要条件。</b>climb 场景内部即有干净对照：t=20.0 s 仅 ref_x 折角、
t=40.0 s 仅 ref_y 折角，两者的倾角指令与 t=30.0 s 完全等价（均饱和 9.5&deg;、速率约 560 &deg;/s、
上升 40 ms），姿态峰却只有 11.92&deg;（放大 1.255）且合规；而 t=30.0 s 是 ref_x 与 ref_y
同时折角，姿态峰 17.39&deg;（放大 1.759）越界。差别在指令之后：t=30 时俯仰与横滚两路内环
同时饱和于 &plusmn;6.1 达 7 个采样点（单轴事件为 0），X 型混控使两轴需求在一对对角旋翼上
同号叠加——电机指令摆幅 u1=18.01 / u2=1.08 / u3=16.82 / u4=1.25，而单轴事件为均衡的 8.4~8.8。</p>
<p>需要说明的是，eight 场景 t=10.0 s 处三轴同时不连续、两路内环也同时饱和，姿态峰却仅
9.41&deg;/12.14&deg; 不越界，故&ldquo;多轴同时饱和&rdquo;亦非充分条件。6 个事件不足以确定充分条件；
本包据此把限速与 CBF 都做成可独立开关的模块，由消融实验在 MWorks 上定量分离各自作用。</p>
<p>另需更正一个易犯的判断：增强版并非&ldquo;内环阻尼不足&rdquo;。在干净单轴事件上辨识
（R&sup2;=0.982~0.995）得 baseline &zeta;=0.348、enhanced &zeta;=0.358，增强版阻尼略高。
把 SISO 二阶模型套在 t=30 双轴耦合事件上会拟出虚假的 &zeta;=0.252（R&sup2; 降至 0.756），
那是模型把无法解释的耦合量吸收成了低阻尼。</p>
<p>原 AdvancedSafetyControl.mo 中的 CBF/PPC/ERG 四个 block 仅有定义、无任何实例化与
connect，未进入控制回路。本包将其真正接入。</p>

<h4>参数取值依据（全部来自实测数据，非经验拍定）</h4>
<ul>
<li><b>ERG 限速 60 deg/s</b>：剔除跳变窗口后，正常飞行的倾角指令变化率
    p99.9 为 6.6~39.3 deg/s、最大 46 deg/s；跳变段为 374~1500 deg/s。
    60 deg/s 高于正常飞行上界、低于跳变激励一个数量级，实测仅 0.06% 的采样点被限住
    （且正是跳变本身）。</li>
<li><b>CBF 制动裕度 kOmega = 0.035 s</b>：实测姿态峰值前 20 ms 的角速率为
    245~330 deg/s，对应超出指令约 7&deg;，即制动距离 ≈ 7/270 ≈ 0.026 s，取 0.035 s 留裕度。</li>
<li><b>参考整形 aMax = 20 m/s²</b>：该值下平稳段引入的滞后 RMS 仅 0.011 m
    （相对 climb/eight 的路径 RMSE 0.23/0.15 m 可忽略），同时可消除 spiral 在
    t=10 s 处 3.0 m 的位置阶跃。</li>
</ul>

<h4>使用方式（两层，可独立启用）</h4>
<p><b>第一层（推荐先跑，零重连线）</b>：用 Task.ClimbSafeERG 等模型，仅通过 redeclare
把 PID3/PID4 换成 ControlMethod.HybridPIDSlidingModeERG，控制器内部接线完全不动。</p>
<p><b>第二层（完整安全链）</b>：用 Task.ClimbSafeFull 等模型，启用 SafeController，
倾角通道串入 ERG 与姿态 CBF，并输出安全统计量。</p>
</html>"));

  package ControlMethod "控制算法（可直接 redeclare 进原 Controller）"
    extends Modelica.Icons.Package;

    model HybridPIDSlidingModeERG
      "复合滑模控制器 + 输出限速(ERG)。与原 HybridPIDSlidingMode 数值一致，仅在输出端加限速"
      parameter Real KP = 1;
      parameter Real KI = 0;
      parameter Real KD = 0;
      parameter Real lambda = 1 "滑模面误差权重";
      parameter Real kSwitch = 0.1 "滑模补偿增益";
      parameter Real boundary = 0.2 "边界层厚度";
      parameter Real uMax = 10 "输出限幅";
      parameter Boolean useERG = true "是否启用输出限速";
      parameter Real rateMax = 10.47
        "输出最大变化率[1/s]。下游gain=0.1，倾角指令限速=0.1*rateMax[rad/s]；10.47对应60deg/s";
      parameter Real tauTrack = 0.01 "限速器跟踪时间常数，取足够小以退化为纯限速";
      extends Modelica.Blocks.Interfaces.SISO;

      Modelica.Blocks.Continuous.Derivative der1;
      Modelica.Blocks.Continuous.Integrator integrator;

      output Real rawCmd "未限速的复合滑模输出";
      output Real rateSat "限速器是否饱和(1/0)，用于统计 ERG 干预次数";
    protected
      Real s "滑模面";
      Real yState(start = 0) "限速后状态";
      Real rateDes "期望变化率";
    equation
      connect(u, der1.u);
      connect(u, integrator.u);
      s = der1.y + lambda * u;
      rawCmd = min(max(KP * u + KI * integrator.y + KD * der1.y
                       + kSwitch * tanh(s / max(boundary, 1e-6)), -uMax), uMax);

      rateDes = (rawCmd - yState) / max(tauTrack, 1e-6);
      der(yState) = noEvent(min(max(rateDes, -rateMax), rateMax));
      rateSat = noEvent(if abs(rateDes) > rateMax then 1.0 else 0.0);

      y = if useERG then yState else rawCmd;
      annotation (Documentation(info = "<html><p>useERG=false 时与原
        QuadrotorModel.Blocks.ControlMethod.HybridPIDSlidingMode 完全等价，便于做对照。</p></html>"));
    end HybridPIDSlidingModeERG;

    model PIDWithERG "基线 PID + 输出限速，用于给 baseline 也做同口径对照"
      parameter Real KP = 1;
      parameter Real KI = 0;
      parameter Real KD = 0;
      parameter Boolean useERG = true;
      parameter Real rateMax = 10.47;
      parameter Real tauTrack = 0.01;
      extends Modelica.Blocks.Interfaces.SISO;

      Modelica.Blocks.Continuous.Derivative der1;
      Modelica.Blocks.Continuous.Integrator integrator;
      output Real rawCmd;
    protected
      Real yState(start = 0);
      Real rateDes;
    equation
      connect(u, der1.u);
      connect(u, integrator.u);
      rawCmd = KP * u + KI * integrator.y + KD * der1.y;
      rateDes = (rawCmd - yState) / max(tauTrack, 1e-6);
      der(yState) = noEvent(min(max(rateDes, -rateMax), rateMax));
      y = if useERG then yState else rawCmd;
    end PIDWithERG;
  end ControlMethod;

  package Blocks "安全链功能块"
    extends Modelica.Icons.Package;

    block TiltRateGovernor "倾角指令显式参考调节器(ERG)：一阶跟踪 + 硬限速"
      parameter Real rateMax = 1.047 "最大指令变化率 [rad/s] (60 deg/s)";
      parameter Real tauTrack = 0.01 "跟踪时间常数";
      extends Modelica.Blocks.Interfaces.SISO;
      output Real governed "限速触发标志(1/0)";
    protected
      Real yState(start = 0);
      Real rateDes;
    equation
      rateDes = (u - yState) / max(tauTrack, 1e-6);
      der(yState) = noEvent(min(max(rateDes, -rateMax), rateMax));
      governed = noEvent(if abs(rateDes) > rateMax then 1.0 else 0.0);
      y = yState;
    end TiltRateGovernor;

    block AttitudeCBF
      "姿态控制障碍函数：把角速率的制动距离计入安全边界，再对倾角指令做最小修正投影"
      parameter Real angleLimit = 15 * Modelica.Constants.pi / 180 "姿态硬边界 [rad]";
      parameter Real kOmega = 0.035 "角速率制动裕度 [s]";
      parameter Real margin = 1.0 * Modelica.Constants.pi / 180 "额外安全余量 [rad]";
      parameter Real omegaFilterT = 0.02 "角速率估计滤波常数 [s]";

      Modelica.Blocks.Interfaces.RealInput cmd "原始倾角指令 [rad]";
      Modelica.Blocks.Interfaces.RealInput angle "实测姿态角 [rad]";
      Modelica.Blocks.Interfaces.RealOutput safeCmd "投影后的安全倾角指令 [rad]";
      Modelica.Blocks.Interfaces.RealOutput hValue "障碍函数值，>0 为安全";
      Modelica.Blocks.Interfaces.RealOutput correction "CBF 修正量绝对值 [rad]";
      Modelica.Blocks.Interfaces.BooleanOutput active "CBF 是否在干预";
    protected
      Real angleFiltered(start = 0);
      Real omega "角速率估计 [rad/s]";
      Real allowed "当前允许的指令上界 [rad]";
    equation
      // 一阶滤波微分估计角速率，避免直接对测量信号求导引入噪声
      der(angleFiltered) = (angle - angleFiltered) / max(omegaFilterT, 1e-6);
      omega = (angle - angleFiltered) / max(omegaFilterT, 1e-6);

      // 障碍函数 h = θmax - |θ| - kΩ·|ω| - margin，计入以当前角速率继续运动的制动距离
      hValue = angleLimit - abs(angle) - kOmega * abs(omega) - margin;

      // 允许的指令幅值随角速率收缩：角速率越大，可用倾角越小
      allowed = max(angleLimit - kOmega * abs(omega) - margin, 0.0);
      safeCmd = min(max(cmd, -allowed), allowed);
      correction = abs(safeCmd - cmd);
      active = noEvent(correction > 1e-9);
      annotation (Documentation(info = "<html>
        <p>姿态角约束对倾角指令是相对度 2 的，直接对 |θ|&le;θmax 做投影无法阻止超调
        （原 AdvancedSafetyControl.CBFCommandFilter 即为此种静态限幅，实测限到 9.5&deg;
        后姿态仍冲到 16.7&deg;）。本块引入角速率项构成扩展障碍函数，使指令上界随角速率
        收缩，从而对超调提供约束。</p></html>"));
    end AttitudeCBF;

    block ReferenceGovernor3D
      "三轴参考可行性整形：时间最优跟踪微分器。斜坡段零稳态滞后，加速度有界"
      parameter Real aMax = 20.0 "最大参考加速度 [m/s2]";
      parameter Real vMax = 8.0 "最大参考速度 [m/s]";
      parameter Real delta = 0.05 "切换边界层，抑制抖振";
      Modelica.Blocks.Interfaces.RealInput u[3] "原始位置指令";
      Modelica.Blocks.Interfaces.RealOutput y[3] "整形后位置指令";
      output Real vel[3] "整形参考速度，可直接用于前馈";
    protected
      Real r[3](each start = 0);
      Real v[3](each start = 0);
      Real sw[3] "时间最优切换函数";
    equation
      for i in 1:3 loop
        sw[i] = (r[i] - u[i]) + v[i] * abs(v[i]) / (2 * aMax);
        der(r[i]) = v[i];
        der(v[i]) = -aMax * tanh(sw[i] / max(delta, 1e-6));
        y[i] = r[i];
        vel[i] = v[i];
      end for;
      annotation (Documentation(info = "<html>
        <p>对阶跃输入按 aMax 限加速度过渡，对斜坡输入收敛后 v 等于斜率、a&rarr;0，
        因此不引入稳态滞后——这是相对普通二阶滤波的关键优势（后者在 1 m/s 匀速段
        会带来 0.1~0.2 m 滞后，与本作品 0.15~0.23 m 的路径 RMSE 同量级）。</p></html>"));
    end ReferenceGovernor3D;

    block SafetyStatistics "安全指标在线统计，替代离线脚本口径"
      parameter Real angleLimit = 15 * Modelica.Constants.pi / 180;
      Modelica.Blocks.Interfaces.RealInput roll;
      Modelica.Blocks.Interfaces.RealInput pitch;
      Modelica.Blocks.Interfaces.RealInput cbfCorrRoll;
      Modelica.Blocks.Interfaces.RealInput cbfCorrPitch;
      output Real attitudeNorm "当前姿态模 [rad]，最大值由后处理取";
      output Real violationTime(start = 0) "超过 15deg 的累计时长 [s]";
      output Real cbfWork(start = 0) "CBF 累计修正量积分 [rad*s]";
      output Boolean inViolation "当前是否超限";
    equation
      attitudeNorm = sqrt(roll ^ 2 + pitch ^ 2);
      inViolation = noEvent(abs(roll) > angleLimit or abs(pitch) > angleLimit);
      der(violationTime) = if inViolation then 1.0 else 0.0;
      der(cbfWork) = abs(cbfCorrRoll) + abs(cbfCorrPitch);
      annotation (Documentation(info = "<html><p>violationTime 是连续量，直接替代原来
        『超限采样点计数』这一依赖采样间隔的口径——同一条轨迹换个 Interval，点数就会变，
        而累计时长不会。</p></html>"));
    end SafetyStatistics;
  end Blocks;

  package Controller "接入安全链的控制器"
    extends Modelica.Icons.Package;

    model SafeSlidingModeController
      "第一层：仅把 PID3/PID4 换成带 ERG 的复合滑模，控制器内部接线完全不动"
      extends QuadrotorModel.Blocks.Controller.Controller(
        redeclare UAVSafetyChain.ControlMethod.HybridPIDSlidingModeERG PID3(
          KP = 2.0, KI = 0.0, KD = 1.35,
          lambda = 1.25, kSwitch = 0.1, boundary = 0.35, uMax = 2.1,
          useERG = true, rateMax = 10.47),
        redeclare UAVSafetyChain.ControlMethod.HybridPIDSlidingModeERG PID4(
          KP = 2.0, KI = 0.0, KD = 1.35,
          lambda = 1.25, kSwitch = 0.1, boundary = 0.35, uMax = 2.1,
          useERG = true, rateMax = 10.47),
        PID1(KP = 6.1, KI = 0.0, KD = 0.12),
        PID5(KP = 17.0, KI = 0.0, KD = 2.05),
        PID6(KP = 17.0, KI = 0.0, KD = 2.05),
        PID7(KP = 10.0, KI = 5.5, KD = 5.0),
        limiter1(uMax = 12.0 / 57.3, uMin = -12.0 / 57.3),
        limiter2(uMax = 12.0 / 57.3, uMin = -12.0 / 57.3),
        limiter3(uMax = 6.1, uMin = -6.1),
        limiter4(uMax = 6.1, uMin = -6.1),
        limiter5(uMax = 6.1, uMin = -6.1));
      annotation (Documentation(info = "<html>
        <p>限幅由原增强版的 9.5&deg; 放回 12&deg;。原方案把限幅从 15&deg; 收紧到 9.5&deg;
        意在提升安全性，实测无效：climb 的 t=20/30/40 三个事件、spiral 与 eight 的跳变事件，
        倾角指令<b>无一例外都顶满限幅</b>，收紧限幅只是让饱和来得更早，没有改变饱和本身，
        却牺牲了机动能力。限制<b>变化率</b>才能让指令不以阶跃形式进入内环，
        既缓解触发条件，也降低两路内环同时饱和的概率。</p></html>"));
    end SafeSlidingModeController;

    model BaselineWithERG "基线 PID + ERG，用于分离『ERG 贡献』与『滑模贡献』"
      extends QuadrotorModel.Blocks.Controller.Controller(
        redeclare UAVSafetyChain.ControlMethod.PIDWithERG PID3(
          KP = 1.5, KI = 0.0, KD = 1.0, useERG = true, rateMax = 10.47),
        redeclare UAVSafetyChain.ControlMethod.PIDWithERG PID4(
          KP = 1.5, KI = 0.0, KD = 1.0, useERG = true, rateMax = 10.47));
    end BaselineWithERG;

    model SafeControllerFull
      "第二层：倾角通道真正串入 ERG 与姿态 CBF 的完整控制器（等价重写原 Controller 拓扑）"
      extends QuadrotorModel.Blocks.Controller.PartialController;

      // ---- 位置外环（与增强版一致）----
      parameter Real KP3 = 2.0 "水平位置环比例";
      parameter Real KD3 = 1.35 "水平位置环微分";
      parameter Real lambda = 1.25 "滑模面权重";
      parameter Real kSwitch = 0.1 "滑模切换增益";
      parameter Real boundary = 0.35 "边界层厚度";
      parameter Real uMaxPos = 2.1 "位置环输出限幅";
      parameter Real tiltGain = 0.1 "位置环输出到倾角指令的换算增益（原 gain5/gain6）";
      parameter Real tiltLimit = 12.0 / 57.3 "倾角指令幅值限幅 [rad]";
      // ---- 高度与姿态内环 ----
      parameter Real KP7 = 10.0, KI7 = 5.5, KD7 = 5.0 "高度环";
      parameter Real KP5 = 17.0, KD5 = 2.05 "俯仰内环";
      parameter Real KP6 = 17.0, KD6 = 2.05 "横滚内环";
      parameter Real KP1 = 6.1, KD1 = 0.12 "偏航内环";
      parameter Real attLimit = 6.1 "姿态环输出限幅（原 limiter3/4/5）";
      parameter Real mixGain = 0.707 "混控增益（原 gain/gain1/gain12）";
      parameter Real Tf = 0.01 "微分滤波常数（对齐 Modelica.Blocks.Continuous.Derivative 默认）";
      // ---- 安全链开关 ----
      parameter Boolean useERG = true "启用倾角指令限速";
      parameter Boolean useCBF = true "启用姿态 CBF";

      UAVSafetyChain.Blocks.TiltRateGovernor ergPitch(rateMax = 1.047);
      UAVSafetyChain.Blocks.TiltRateGovernor ergRoll(rateMax = 1.047);
      UAVSafetyChain.Blocks.AttitudeCBF cbfPitch;
      UAVSafetyChain.Blocks.AttitudeCBF cbfRoll;

      output Real pitchCmdRaw "限速/CBF 之前的倾角指令 [rad]";
      output Real rollCmdRaw;
      output Real pitchCmdSafe "进入姿态内环的最终指令 [rad]";
      output Real rollCmdSafe;
      output Real cbfCorrPitch "CBF 修正量 [rad]";
      output Real cbfCorrRoll;
    protected
      Real ex, ey, ez "位置误差";
      Real dxs(start = 0), dys(start = 0), dzs(start = 0) "微分滤波状态";
      Real dex, dey, dez "滤波微分";
      Real iz(start = 0) "高度积分";
      Real sx, sy "滑模面";
      Real pid3out, pid4out, thrust;
      Real ePitch, eRoll, eYaw;
      Real dps(start = 0), drs(start = 0), dys2(start = 0);
      Real dePitch, deRoll, deYaw;
      Real gp, gr, gy;
      Real m1, m2, m3, m4;
    equation
      // ================= 位置外环 =================
      ex = position_command[1] - position[1];
      ey = position_command[2] - position[2];
      ez = position_command[3] - position[3];

      der(dxs) = (ex - dxs) / Tf;  dex = (ex - dxs) / Tf;
      der(dys) = (ey - dys) / Tf;  dey = (ey - dys) / Tf;
      der(dzs) = (ez - dzs) / Tf;  dez = (ez - dzs) / Tf;
      der(iz) = ez;

      sx = dex + lambda * ex;
      sy = dey + lambda * ey;
      pid3out = min(max(KP3 * ex + KD3 * dex + kSwitch * tanh(sx / boundary), -uMaxPos), uMaxPos);
      pid4out = min(max(KP3 * ey + KD3 * dey + kSwitch * tanh(sy / boundary), -uMaxPos), uMaxPos);

      pitchCmdRaw = min(max(tiltGain * pid3out, -tiltLimit), tiltLimit);
      rollCmdRaw  = min(max(tiltGain * pid4out, -tiltLimit), tiltLimit);

      // ================= 安全链：ERG 限速 -> 姿态 CBF 投影 =================
      ergPitch.u = pitchCmdRaw;
      ergRoll.u  = rollCmdRaw;
      cbfPitch.cmd   = if useERG then ergPitch.y else pitchCmdRaw;
      cbfRoll.cmd    = if useERG then ergRoll.y  else rollCmdRaw;
      cbfPitch.angle = angle[2];
      cbfRoll.angle  = -angle[1];
      pitchCmdSafe = if useCBF then cbfPitch.safeCmd else cbfPitch.cmd;
      rollCmdSafe  = if useCBF then cbfRoll.safeCmd  else cbfRoll.cmd;
      cbfCorrPitch = cbfPitch.correction;
      cbfCorrRoll  = cbfRoll.correction;

      // ================= 姿态内环（原 gain13 = -1 的符号约定保留）=================
      ePitch = pitchCmdSafe - angle[2];
      eRoll  = rollCmdSafe - (-angle[1]);
      eYaw   = 0.0 - angle[3];

      der(dps)  = (ePitch - dps) / Tf;   dePitch = (ePitch - dps) / Tf;
      der(drs)  = (eRoll - drs) / Tf;    deRoll  = (eRoll - drs) / Tf;
      der(dys2) = (eYaw - dys2) / Tf;    deYaw   = (eYaw - dys2) / Tf;

      gp = mixGain * min(max(KP5 * ePitch + KD5 * dePitch, -attLimit), attLimit);
      gr = mixGain * min(max(KP6 * eRoll + KD6 * deRoll, -attLimit), attLimit);
      gy = mixGain * min(max(KP1 * eYaw + KD1 * deYaw, -attLimit), attLimit);
      thrust = KP7 * ez + KI7 * iz + KD7 * dez;

      // ================= 混控（严格照抄原 add3_1..4 的符号与 gain8..11）=================
      m1 = -gy - gp + gr;
      m2 =  gy - gp - gr;
      m3 = -gy + gp - gr;
      m4 =  gy + gp + gr;
      y  =  (m1 + thrust);
      y1 = -(m2 + thrust);
      y2 =  (m3 + thrust);
      y3 = -(m4 + thrust);

      annotation (Documentation(info = "<html>
        <p>本模型按原 QuadrotorModel.Blocks.Controller.Controller 的连线逐条等价重写
        （混控符号取自 add3_1~add3_4 的 k1/k2/k3 与 gain8~gain11 的 ±1，
        横滚反馈保留 gain13=-1 的符号约定），唯一改动是在倾角指令通道串入
        ERG 与姿态 CBF。</p>
        <p><b>回归验证方法</b>：置 useERG=false 且 useCBF=false，
        并把 tiltLimit 改回 9.5/57.3，本模型应与
        QuadrotorModel.Task.ClimbEnhanced 输出一致。请队友先跑这一步确认等价性，
        再开启安全链——不要跳过这个回归。</p></html>"));
    end SafeControllerFull;
  end Controller;

  package Task "对照实验任务模型"
    extends Modelica.Icons.Package;

    // ---------- 第一层：ERG（零重连线，建议先跑这组） ----------
    model ClimbSafeERG "爬升 + 复合滑模 + ERG"
      extends QuadrotorModel.Examples.Example1(
        redeclare UAVSafetyChain.Controller.SafeSlidingModeController controller3_2);
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 50,
        Tolerance = 0.0001, Interval = 0.01));
    end ClimbSafeERG;

    model SpiralSafeERG "螺旋爬升 + 复合滑模 + ERG"
      extends QuadrotorModel.Examples.Example2(
        redeclare UAVSafetyChain.Controller.SafeSlidingModeController controller3_2);
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 50,
        Tolerance = 0.0001, Interval = 0.01));
    end SpiralSafeERG;

    model EightSafeERG "8字航迹 + 复合滑模 + ERG"
      extends QuadrotorModel.Examples.Example3(
        redeclare UAVSafetyChain.Controller.SafeSlidingModeController controller3_2);
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 120,
        Tolerance = 0.0001, Interval = 0.01));
    end EightSafeERG;

    // ---------- 消融对照：只加 ERG 不换控制器 ----------
    model ClimbBaselineERG "爬升 + 基线PID + ERG（分离 ERG 的独立贡献）"
      extends QuadrotorModel.Examples.Example1(
        redeclare UAVSafetyChain.Controller.BaselineWithERG controller3_2);
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 50,
        Tolerance = 0.0001, Interval = 0.01));
    end ClimbBaselineERG;

    model SpiralBaselineERG "螺旋 + 基线PID + ERG"
      extends QuadrotorModel.Examples.Example2(
        redeclare UAVSafetyChain.Controller.BaselineWithERG controller3_2);
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 50,
        Tolerance = 0.0001, Interval = 0.01));
    end SpiralBaselineERG;

    // ---------- 等价性回归：必须先跑这个 ----------
    model ClimbEquivalenceCheck
      "回归验证：关闭安全链的 SafeControllerFull 应与 QuadrotorModel.Task.ClimbEnhanced 一致"
      extends QuadrotorModel.Examples.Example1(
        redeclare UAVSafetyChain.Controller.SafeControllerFull controller3_2(
          useERG = false, useCBF = false, tiltLimit = 9.5 / 57.3));
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 50,
        Tolerance = 0.0001, Interval = 0.01),
        Documentation(info = "<html><p>把本模型的 path_rmse 与 ClimbEnhanced 的
        0.23289 对比。若不一致，说明等价重写有误，此时后续所有安全链结论都不可信，
        必须先修好再往下走。</p></html>"));
    end ClimbEquivalenceCheck;

    // ---------- 第二层：完整安全链 ----------
    model ClimbSafeFull "爬升 + 完整安全链(ERG+CBF)"
      extends QuadrotorModel.Examples.Example1(
        redeclare UAVSafetyChain.Controller.SafeControllerFull controller3_2);
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 50,
        Tolerance = 0.0001, Interval = 0.01));
    end ClimbSafeFull;

    model SpiralSafeFull "螺旋爬升 + 完整安全链"
      extends QuadrotorModel.Examples.Example2(
        redeclare UAVSafetyChain.Controller.SafeControllerFull controller3_2);
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 50,
        Tolerance = 0.0001, Interval = 0.01));
    end SpiralSafeFull;

    model EightSafeFull "8字航迹 + 完整安全链"
      extends QuadrotorModel.Examples.Example3(
        redeclare UAVSafetyChain.Controller.SafeControllerFull controller3_2);
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 120,
        Tolerance = 0.0001, Interval = 0.01));
    end EightSafeFull;

    // ---------- 消融：只开 CBF 不开 ERG / 只开 ERG 不开 CBF ----------
    model ClimbERGOnly "爬升 + 仅 ERG"
      extends QuadrotorModel.Examples.Example1(
        redeclare UAVSafetyChain.Controller.SafeControllerFull controller3_2(
          useERG = true, useCBF = false));
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 50,
        Tolerance = 0.0001, Interval = 0.01));
    end ClimbERGOnly;

    model ClimbCBFOnly "爬升 + 仅 CBF"
      extends QuadrotorModel.Examples.Example1(
        redeclare UAVSafetyChain.Controller.SafeControllerFull controller3_2(
          useERG = false, useCBF = true));
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 50,
        Tolerance = 0.0001, Interval = 0.01));
    end ClimbCBFOnly;
  end Task;
end UAVSafetyChain;
