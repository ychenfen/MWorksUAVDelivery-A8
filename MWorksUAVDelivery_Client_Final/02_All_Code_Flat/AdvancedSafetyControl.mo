within;
package AdvancedSafetyControl "智能安全控制补充模块"
  extends Modelica.Icons.Package;

  block PPCEnvelope "预设性能控制误差包络"
    parameter Real rho0 = 1.0 "初始误差包络";
    parameter Real rhoInf = 0.05 "稳态误差包络";
    parameter Real lambda = 0.15 "收敛速度";
    Modelica.Blocks.Interfaces.RealInput e "跟踪误差";
    Modelica.Blocks.Interfaces.RealOutput rho "误差边界";
    Modelica.Blocks.Interfaces.RealOutput xi "归一化误差";
  equation
    rho = rhoInf + (rho0 - rhoInf) * exp(-lambda * time);
    xi = e / max(rho, 1e-6);
  end PPCEnvelope;

  block CBFCommandFilter "姿态安全边界投影滤波器"
    parameter Real angleLimit = 15 * Modelica.Constants.pi / 180 "姿态角边界";
    Modelica.Blocks.Interfaces.RealInput rollCmd;
    Modelica.Blocks.Interfaces.RealInput pitchCmd;
    Modelica.Blocks.Interfaces.RealOutput rollSafe;
    Modelica.Blocks.Interfaces.RealOutput pitchSafe;
    Modelica.Blocks.Interfaces.RealOutput correctionNorm;
  equation
    rollSafe = min(max(rollCmd, -angleLimit), angleLimit);
    pitchSafe = min(max(pitchCmd, -angleLimit), angleLimit);
    correctionNorm = sqrt((rollSafe - rollCmd) ^ 2 + (pitchSafe - pitchCmd) ^ 2);
  end CBFCommandFilter;

  block ERGReferenceGovernor "显式参考调节器一阶限速模型"
    parameter Real tau = 0.35 "参考跟随时间常数";
    parameter Real rateLimit = 2.0 "最大参考变化率";
    Modelica.Blocks.Interfaces.RealInput refCmd;
    Modelica.Blocks.Interfaces.RealOutput refOut(start = 0);
  protected
    Real rawRate;
  equation
    rawRate = (refCmd - refOut) / max(tau, 1e-6);
    der(refOut) = min(max(rawRate, -rateLimit), rateLimit);
  end ERGReferenceGovernor;

  block AttitudeRiskMonitor "轻量时序姿态风险预警"
    parameter Real angleLimit = 15 * Modelica.Constants.pi / 180;
    parameter Real rateLimit = 2.0;
    Modelica.Blocks.Interfaces.RealInput roll;
    Modelica.Blocks.Interfaces.RealInput pitch;
    Modelica.Blocks.Interfaces.RealInput rollRate;
    Modelica.Blocks.Interfaces.RealInput pitchRate;
    Modelica.Blocks.Interfaces.RealOutput riskScore;
    Modelica.Blocks.Interfaces.BooleanOutput warning;
  equation
    riskScore = max(abs(roll), abs(pitch)) / angleLimit + 0.2 * max(abs(rollRate), abs(pitchRate)) / rateLimit;
    warning = riskScore > 0.9;
  end AttitudeRiskMonitor;

  model AdvancedClimbSafetyDemo "爬升场景高级安全控制演示模型"
    extends QuadrotorModel.Task.ClimbEnhanced;
    annotation (Documentation(info = "<html><p>该模型继承增强爬升模型。CBF/PPC/ERG 等补充模块见本包中的 block 定义，批量验证由 advanced_safety_intelligence_suite.py 输出。</p></html>"));
  end AdvancedClimbSafetyDemo;
end AdvancedSafetyControl;
