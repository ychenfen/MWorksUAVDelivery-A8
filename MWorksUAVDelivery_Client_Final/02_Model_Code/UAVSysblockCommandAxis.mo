model UAVSysblockCommandAxis
  extends ModelWorkspace;
  import SysplorerEmbeddedCoder.Types.*;
  import BaseWorkspace.*;
  annotation (__MWORKS(version="26.2.1",modelType = Control,PortArrangement(Left(cmd_in), Right(cmd_out)),BlockSystem(blockKind=BlockKind.userModel,SampleTime(auto=true,group="")=0.02,OutputInterval=0.02),SysblockVersion = "1.0"),
    Icon(coordinateSystem(preserveAspectRatio = false)),
    experiment(Algorithm = Euler, Interval = -1),
    Documentation(info = "<html><p>Standalone Sysblock user-model sample for a single UAV command axis. This file is delivered to prove direct Sysblock platform usage in MWORKS/Sysplorer.</p></html>"));

  SysplorerEmbeddedCoder.Port.Inport cmd_in 
    annotation (Placement(transformation(origin = {-180, 36}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {-101.8, 0}, extent = {{-1.8, -1.8}, {1.8, 1.8}})),__MWORKS(BlockSystem(SampleTime(group="D0")=0,Type(ref="double"),Dimension=1)));
  AxisSubSystem axisSubSystem 
    annotation (Placement(transformation(origin = {-22, 36}, extent = {{-61, -28}, {61, 28}})),
      __MWORKS(PortLabels(labelType = "PortName")));
  SysplorerEmbeddedCoder.Port.Outport cmd_out 
    annotation (Placement(transformation(origin = {142, 36}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {101.8, 0}, extent = {{-1.8, -1.8}, {1.8, 1.8}})),__MWORKS(BlockSystem(SampleTime(group="D0")=0,Type(ref="double"),Dimension=1)));

  block AxisSubSystem
    SysplorerEmbeddedCoder.Port.Inport u 
      annotation (Placement(transformation(origin = {-158, -0.845782}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-301.8, 0}, extent = {{-1.8, -1.8}, {1.8, 1.8}})),__MWORKS(BlockSystem(SampleTime(group="D0")=0,Type(ref="double"),Dimension=1)));
    SysplorerEmbeddedCoder.Port.Outport y 
      annotation (Placement(transformation(origin = {156, -0.845782}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {301.8, 0}, extent = {{-1.8, -1.8}, {1.8, 1.8}})),__MWORKS(BlockSystem(SampleTime(group="D0")=0,Type(ref="double"),Dimension=1)));
    SysplorerEmbeddedCoder.MathOperation.Gain gain 
      annotation (Placement(transformation(origin = {-1, -0.845777}, extent = {{-10, -10}, {10, 10}})),__MWORKS(BlockSystem(Instance(u(Type(ref="double"),Dimension=1),
y(Type(ref="double"),Dimension=1),
k(Type(ref="double"),Dimension=1)),SampleTime(group="D0")=0)));
    annotation (defaultComponentName = "axisSubSystem",
      __MWORKS(
        PortArrangement(Left(u), Right(y)),
        BlockSystem(
          blockKind = BlockKind.subSystem,
          SampleTime(auto = true),
          SubSystem(virtual = true, functionPack = FunctionPack.auto, functionName = "", sourceFile = "")),
        PortLabels(labelType = "PortName"),
        sourceModel = SysplorerEmbeddedCoder.SubSystems.SubSystem,
        independentInstance = true,
        hide = true),
      Icon(coordinateSystem(extent = {{-300, -120}, {300, 120}}, grid = {2, 2}), graphics = {
          Rectangle(sizePolicy = SizePolicy.Expanding, rotationPolicy = RotationPolicy.Follow, origin = {0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 3, extent = {{-300, 120}, {300, -120}}),
          Text(origin = {0, -140}, lineColor = {0, 0, 0}, extent = {{0, -20}, {0, 20}}, textString = "%name", fontSize = 14, textStyle = {TextStyle.None}, textColor = {0, 0, 0}, verticalAlignment = TextAlignment.Top)},
        sizePolicy = SizePolicy.Fixed,
        rotationPolicy = RotationPolicy.Ignore),
      Protection(access = Access.packageDuplicate));
  equation
    connect(u, gain.u) 
      annotation (Line(origin = {-28, -1}, points = {{-15.8426, 0.154223}, {15.2303, 0.154223}}, color = {0, 0, 0}));
    connect(gain.y, y) 
      annotation (Line(origin = {28, -1}, points = {{-17.1697, 0.154223}, {16.6884, 0.154223}}, color = {0, 0, 0}));
  end AxisSubSystem;
  model ModelWorkspace
    annotation(__MWORKS(hide = true,BlockSystem(blockKind=BlockKind.modelWorkspace)));
  end ModelWorkspace;

equation
  connect(cmd_in, axisSubSystem.u) 
    annotation (Line(origin = {-126, 36}, points = {{-42, 0}, {41.2, 0}}, color = {0, 0, 0}));
  connect(axisSubSystem.y, cmd_out) 
    annotation (Line(origin = {85, 36}, points = {{-44.2, 0}, {45, 0}}, color = {0, 0, 0}));
end UAVSysblockCommandAxis;