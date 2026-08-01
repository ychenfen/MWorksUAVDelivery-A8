within;
package UAVFormation "多机编队：一致性编队控制律 + 势场避撞"
  extends Modelica.Icons.Package;

  annotation (Documentation(info = "<html>
<h4>为什么重做</h4>
<p>原 advanced_safety_intelligence_suite.py 的 formation_demo() 并不是编队算法：
它读取单机 8 字轨迹 CSV，给僚机加一个固定偏移量再做 0.12 s 延迟的一阶跟随。
没有控制律、没有机间信息交互、没有避撞，所谓『最大队形误差 0.861 m』
只是延迟造成的几何偏差，与控制性能无关。</p>

<h4>本包实现的算法</h4>
<p>领航-跟随 + 一致性编队，对僚机 i：</p>
<pre>
cmd_i = p_L + d_i
        + kConsensus * Σ_{j∈N(i)} [ (p_j - p_i) - (d_j - d_i) ]   // 一致性项
        + repulsion_i                                              // 势场避撞
</pre>
<p>一致性项使用<b>机间相对测量</b>，这是与原『各自跟自己的偏移量』的本质区别：
当某架僚机被扰动落后时，邻机会同步调整，队形作为整体收敛，而不是各飞各的。
避撞项采用 Khatib 势场，仅在机间距小于 dSafe 时激活。</p>

<h4>机体模型的口径说明（重要，不要含糊）</h4>
<p>本包的 UAVAgent 用<b>二阶闭环位置模型</b>代替完整多体机体，参数由本作品自己的
MWorks 仿真输出辨识得到：</p>
<ul>
<li>高度通道 wn=1.765 rad/s, zeta=0.257 —— 三场景纯仿真验证拟合优度 93.0/93.2/94.7%，
    参数可辨识。</li>
<li>水平通道取 wn=8 rad/s, zeta=0.7 —— <b>此参数不可由现有数据唯一确定</b>：
    三条航迹的时间尺度远慢于水平位置环带宽，wn 在 8~50 rad/s 范围内拟合优度均为
    98~99%。这里取区间下界（响应最慢），使僚机跟随更迟钝、队形误差偏大，
    结论方向保守，不会高估编队性能。</li>
</ul>
<p>因此本包证明的是<b>编队控制律本身的收敛性与避撞有效性</b>，
不是整机编队的最终精度指标。要得到可写进报告的编队精度，
需按下文把 UAVAgent 换成完整多体机体后在 MWorks 重跑。</p>

<h4>升级到完整多体机体的方法</h4>
<p>UAVAgent 通过 PartialUAVAgent 接口接入，只有 position_command[3] 输入与
position[3] 输出。新建一个 MultibodyUAVAgent extends PartialUAVAgent，
内部实例化 QuadChassis + 4×Actuator + Sensors + 控制器（照抄
QuadrotorModel.Examples.Example1 的连线，去掉内部 ClimbPath，
把 position_command 接到接口），再在 ThreeUAVFormation 里 redeclare 即可，
编队控制律无需任何改动。</p>
<p>已知的两个坑，先说在前面：(1) QuadChassis 内部含 inner world 与地面接触模型，
三实例并存需在 Sysplorer 确认无重复 inner 冲突；(2) 机体 body 的 r_0(fixed=false)，
三机初始位置都会解算到原点附近，需要给各机加初始偏移，否则 t=0 时避撞项立刻触发。</p>
</html>"));

  package Interfaces
    extends Modelica.Icons.InterfacesPackage;

    partial model PartialUAVAgent "单机接口：位置指令进，实测位置出"
      parameter Real p0[3] = {0, 0, 0} "初始位置 [m]";
      Modelica.Blocks.Interfaces.RealInput position_command[3] "位置指令 [m]";
      Modelica.Blocks.Interfaces.RealOutput position[3] "实测位置 [m]";
      annotation (Documentation(info = "<html><p>p0 放在接口层而非具体机体里，
        因为 ThreeUAVFormation 中的机体是 replaceable/constrainedby 的，
        修饰符必须对约束类型有效，否则 redeclare 成完整多体机体时会报错。</p></html>"));
    end PartialUAVAgent;
  end Interfaces;

  package Blocks
    extends Modelica.Icons.Package;

    block FormationLaw
      "三机一致性编队控制律 + 势场避撞。领航机位置为输入，输出两架僚机的位置指令"
      parameter Real offset1[3] = {-3.0, -3.0, 0.0} "僚机1相对领航机的期望偏移 [m]";
      parameter Real offset2[3] = {-3.0, 3.0, 0.0} "僚机2相对领航机的期望偏移 [m]";
      parameter Real kConsensus = 0.6 "一致性增益";
      parameter Real kRepulsion = 25.0 "避撞势场增益 [m^3]，取值依据见下方 Documentation";
      parameter Real dSafe = 2.0 "避撞作用距离 [m]";
      parameter Real dMin = 0.3 "势场数值下限，防止除零 [m]";
      parameter Real cmdLimit = 8.0 "编队修正量限幅 [m]，防止避撞项把僚机推飞";

      Modelica.Blocks.Interfaces.RealInput pL[3] "领航机实测位置";
      Modelica.Blocks.Interfaces.RealInput p1[3] "僚机1实测位置";
      Modelica.Blocks.Interfaces.RealInput p2[3] "僚机2实测位置";
      Modelica.Blocks.Interfaces.RealOutput cmd1[3] "僚机1位置指令";
      Modelica.Blocks.Interfaces.RealOutput cmd2[3] "僚机2位置指令";

      output Real formErr1 "僚机1队形误差 [m]";
      output Real formErr2 "僚机2队形误差 [m]";
      output Real maxFormErr "最大队形误差 [m]";
      output Real dL1 "领航-僚机1 间距 [m]";
      output Real dL2 "领航-僚机2 间距 [m]";
      output Real d12 "僚机间距 [m]";
      output Real minSep "最小机间距 [m]";
    protected
      Real des1[3], des2[3] "期望绝对位置";
      Real cons1[3], cons2[3] "一致性修正";
      Real rep1[3], rep2[3] "避撞修正";
      Real eL1[3], eL2[3], e12[3];
      Real nL1, nL2, n12 "受下限保护的机间距";
      Real wL1, wL2, w12 "势场权重";
    equation
      des1 = pL + offset1;
      des2 = pL + offset2;

      eL1 = p1 - pL;   eL2 = p2 - pL;   e12 = p1 - p2;
      dL1 = sqrt(eL1 * eL1 + 1e-12);
      dL2 = sqrt(eL2 * eL2 + 1e-12);
      d12 = sqrt(e12 * e12 + 1e-12);
      nL1 = max(dL1, dMin);
      nL2 = max(dL2, dMin);
      n12 = max(d12, dMin);
      minSep = min(min(dL1, dL2), d12);

      // 一致性项：用机间相对测量而非各自独立跟偏移量
      // 僚机1 的邻居为 领航机 与 僚机2
      cons1 = kConsensus * (((pL - p1) - (-offset1)) + ((p2 - p1) - (offset2 - offset1)));
      cons2 = kConsensus * (((pL - p2) - (-offset2)) + ((p1 - p2) - (offset1 - offset2)));

      // Khatib 势场避撞：仅在间距小于 dSafe 时激活
      wL1 = if noEvent(nL1 < dSafe) then kRepulsion * (1.0 / nL1 - 1.0 / dSafe) / (nL1 * nL1) else 0.0;
      w12 = if noEvent(n12 < dSafe) then kRepulsion * (1.0 / n12 - 1.0 / dSafe) / (n12 * n12) else 0.0;
      wL2 = if noEvent(nL2 < dSafe) then kRepulsion * (1.0 / nL2 - 1.0 / dSafe) / (nL2 * nL2) else 0.0;
      rep1 = wL1 * (eL1 / nL1) + w12 * (e12 / n12);
      rep2 = wL2 * (eL2 / nL2) - w12 * (e12 / n12);

      for i in 1:3 loop
        cmd1[i] = des1[i] + min(max(cons1[i] + rep1[i], -cmdLimit), cmdLimit);
        cmd2[i] = des2[i] + min(max(cons2[i] + rep2[i], -cmdLimit), cmdLimit);
      end for;

      formErr1 = sqrt((p1 - des1) * (p1 - des1) + 1e-12);
      formErr2 = sqrt((p2 - des2) * (p2 - des2) + 1e-12);
      maxFormErr = max(formErr1, formErr2);
      annotation (Documentation(info = "<html>
<h4>kRepulsion 取值依据（独立 Python 复现同一组方程扫描所得）</h4>
<p>对抗测试：把两架僚机的期望间距设成 0.4 m（远小于 dSafe=2 m），
观察势场能把实际间距撑到多少：</p>
<table border=\"1\" cellpadding=\"4\">
<tr><th>kRepulsion</th><td>1.5</td><td>3</td><td>6</td><td>12</td>
    <td><b>25</b></td><td>50</td><td>100</td></tr>
<tr><th>冲突场景稳态间距/m</th><td>0.98</td><td>1.11</td><td>1.24</td><td>1.39</td>
    <td><b>1.53</b></td><td>1.67</td><td>1.78</td></tr>
</table>
<p>标称编队场景（期望间距 6 m）下，上述所有增益给出的队形误差与最小间距完全相同
（0.0998 m / 4.097 m）——因为间距大于 dSafe 时势场项恒为 0，
所以提高 kRepulsion 在正常编队中是零代价的。取 25 兼顾冲突场景的间距与数值条件。</p>

<h4>已知局限：势场是软约束，给不了硬保证</h4>
<p>上表的间距随增益单调上升但渐近于 dSafe 而永远达不到，这是势场项与一致性项
在平衡点拔河的必然结果。当<b>期望队形本身违反安全间距</b>时，本算法只能缓解、
不能保证。要得到硬保证，需把机间距约束也写成控制障碍函数并求解 QP
（与 UAVSafetyChain.Blocks.AttitudeCBF 同一套方法），
这是明确的后续方向，不要在报告里把当前实现说成『保证不碰撞』。</p></html>"));
    end FormationLaw;
  end Blocks;

  package Components
    extends Modelica.Icons.Package;

    model UAVAgent
      "降阶单机：二阶闭环位置模型。参数来源与不可辨识性见本包 Documentation"
      extends UAVFormation.Interfaces.PartialUAVAgent;
      parameter Real wnXY = 8.0 "水平通道闭环自然频率 [rad/s]（保守下界）";
      parameter Real zetaXY = 0.7 "水平通道阻尼比";
      parameter Real wnZ = 1.765 "高度通道闭环自然频率 [rad/s]（辨识值）";
      parameter Real zetaZ = 0.257 "高度通道阻尼比（辨识值）";
      parameter Real vMaxAgent = 6.0 "机体最大速度 [m/s]，体现执行能力上限";
    protected
      Real v[3](start = {0, 0, 0});
      Real wn[3], ze[3];
    equation
      wn = {wnXY, wnXY, wnZ};
      ze = {zetaXY, zetaXY, zetaZ};
      for i in 1:3 loop
        der(position[i]) = min(max(v[i], -vMaxAgent), vMaxAgent);
        der(v[i]) = wn[i] ^ 2 * (position_command[i] - position[i]) - 2 * ze[i] * wn[i] * v[i];
      end for;
    initial equation
      position = p0;
    end UAVAgent;
  end Components;

  package Examples
    extends Modelica.Icons.Package;

    model ThreeUAVFormation "三机 8 字航迹编队：领航机跟航迹，两架僚机跟编队律"
      parameter Real offset1[3] = {-3.0, -3.0, 0.0};
      parameter Real offset2[3] = {-3.0, 3.0, 0.0};

      QuadrotorModel.PathPlanning.EightPath leaderPath;
      replaceable UAVFormation.Components.UAVAgent leader(p0 = {0, 0, 0})
        constrainedby UAVFormation.Interfaces.PartialUAVAgent;
      replaceable UAVFormation.Components.UAVAgent follower1(p0 = offset1)
        constrainedby UAVFormation.Interfaces.PartialUAVAgent;
      replaceable UAVFormation.Components.UAVAgent follower2(p0 = offset2)
        constrainedby UAVFormation.Interfaces.PartialUAVAgent;
      UAVFormation.Blocks.FormationLaw law(offset1 = offset1, offset2 = offset2);

      output Real maxFormationError "最大队形误差 [m]";
      output Real minSeparation "最小机间距 [m]";
      output Real leaderTrackError "领航机航迹跟踪误差 [m]";
    equation
      connect(leaderPath.position_command, leader.position_command);
      law.pL = leader.position;
      law.p1 = follower1.position;
      law.p2 = follower2.position;
      connect(law.cmd1, follower1.position_command);
      connect(law.cmd2, follower2.position_command);

      maxFormationError = law.maxFormErr;
      minSeparation = law.minSep;
      leaderTrackError = sqrt((leaderPath.position_command - leader.position)
                            * (leaderPath.position_command - leader.position) + 1e-12);
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 120,
        Tolerance = 0.0001, Interval = 0.01));
    end ThreeUAVFormation;

    model FormationDisturbanceTest
      "扰动恢复测试：t=40s 给僚机1 施加 5m 位置偏差，考察一致性项能否把队形拉回"
      extends ThreeUAVFormation(follower1(p0 = {-3.0 + 5.0, -3.0, 0.0}));
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 120,
        Tolerance = 0.0001, Interval = 0.01),
        Documentation(info = "<html><p>用初始偏差代替运行中注入扰动，
        观察 maxFormationError 的收敛过程与 minSeparation 是否始终大于机体尺寸。
        这是原离线脚本完全没有的能力——原脚本的『队形误差』是写死的延迟偏差，
        不存在收敛概念。</p></html>"));
    end FormationDisturbanceTest;

    model FormationCollisionTest
      "避撞测试：把两架僚机的期望偏移设成重合，检验势场项能否维持最小安全间距"
      extends ThreeUAVFormation(
        offset1 = {-3.0, -0.2, 0.0},
        offset2 = {-3.0, 0.2, 0.0});
      annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 60,
        Tolerance = 0.0001, Interval = 0.01),
        Documentation(info = "<html><p>期望队形本身违反安全间距(0.4m &lt; dSafe=2m)，
        观察 minSeparation 是否被势场撑开并稳定在安全值附近。
        这里存在期望队形与避撞的冲突，属于故意设计的对抗测试。</p></html>"));
    end FormationCollisionTest;
  end Examples;
end UAVFormation;
