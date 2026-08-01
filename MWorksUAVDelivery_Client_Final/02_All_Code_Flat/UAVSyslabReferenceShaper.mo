within;
package UAVSyslabReferenceShaper
  extends Modelica.Icons.Package;

  package ImportedTypes
    model UAVRefShaper
      "Syslab function block for shaping UAV position commands"
      annotation (__MWorks(SyslabFunction(
            Type = "function",
            AllFuncNames = "uav_ref_shaper,clamp_cmd",
            Duplicated = true,
            BlockPort(
              in_cmd(Scope = Input, Type = 0, Dims = {3}, Value = 1, Desc = ""),
              out_cmd(Scope = Output, Type = 0, Dims = {3}, Value = 1, Desc = "")))),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, grid = {2, 2})),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, grid = {2, 2}), graphics = {
            Bitmap(origin = {0, 0}, extent = {{-100, -100}, {100, 100}}, fileName = "modelica://SyslabWorkspace/Resources/Images/FunctionAPI.svg"),
            Text(origin = {0, 130}, lineColor = {0, 0, 255}, extent = {{-150, 20}, {150, -20}}, textString = "%name", textColor = {0, 0, 255})}),
        Documentation(info = "<html><p>Standalone Syslab function block for UAV reference shaping. The input is a 3-axis command vector and the output is a bounded command vector for direct use in MWORKS/Sysplorer demos.</p></html>"));
      import Modelica;
      import SyslabWorkspace.Communication;
      extends SyslabWorkspace.Communication.SyslabSampleBase;

      Communication.SyslabFunctionBase base(
        funcName = "uav_ref_shaper",
        scriptText = "base64=ZnVuY3Rpb24gY2xhbXBfY21kKHYsIGxvLCBoaSkKICAgIHJldHVybiBtaW4obWF4KHYsIGxvKSwgaGkpCmVuZAoKZnVuY3Rpb24gdWF2X3JlZl9zaGFwZXIoY21kKQogICAgc2hhcGVkX3ggPSAwLjkwICogY2xhbXBfY21kKGNtZFsxXSwgLTEyLjAsIDEyLjApCiAgICBzaGFwZWRfeSA9IDAuOTAgKiBjbGFtcF9jbWQoY21kWzJdLCAtMTIuMCwgMTIuMCkKICAgIHNoYXBlZF96ID0gY2xhbXBfY21kKGNtZFszXSwgMC4wLCAxOC4wKQogICAgcmV0dXJuIFtzaGFwZWRfeCwgc2hhcGVkX3ksIHNoYXBlZF96XQplbmQ=",
        inputDims = {{3}},
        inputTypes = {0},
        outputDims = {{3}},
        outputTypes = {0},
        startTime = startTime,
        period = period,
        hasInput = true,
        hasOutput = true)
        annotation (Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {0, 0})));
      Modelica.Blocks.Interfaces.RealInput in_cmd[3]
        annotation (Placement(transformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}})));
      ArrayConverter._A2V_1D_Real in_cmd_converter(dims = {3})
        annotation (HideResult = true, Placement(transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput out_cmd[3]
        annotation (Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
      ArrayConverter._V2A_1D_Real out_cmd_converter(dims = {3})
        annotation (HideResult = true, Placement(transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}})));
    equation
      connect(in_cmd, in_cmd_converter.u)
        annotation (Line(origin = {-95, 0}, points = {{-15, 0}, {0, 0}, {0, 0}, {15, 0}}, color = {255, 127, 0}));
      connect(in_cmd_converter.y, base.inputs[1:3])
        annotation (Line(origin = {-44, 32}, points = {{-36, -32}, {0, -32}, {0, 33}, {36, 33}}, color = {255, 127, 0}));
      connect(out_cmd, out_cmd_converter.y)
        annotation (Line(origin = {95, 0}, points = {{15, 0}, {0, 0}, {0, 0}, {-15, 0}}, color = {255, 127, 0}));
      connect(out_cmd_converter.u, base.outputs[1:3])
        annotation (Line(origin = {36, 32}, points = {{44, -32}, {0, -32}, {0, 33}, {-44, 33}}, color = {255, 127, 0}));
    end UAVRefShaper;

    package ArrayConverter
      model _A2V_1D_Real
        "1 dimension Real array to Real vector"
        import Modelica;
        parameter Integer dims[1] = {2};
        Modelica.Blocks.Interfaces.RealInput u[dims[1]]
          annotation (Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealOutput y[product(dims)]
          annotation (Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
      protected
        Integer pos;
      algorithm
        pos := 1;
        for i1 in 1:dims[1] loop
          y[pos] := u[i1];
          pos := pos + 1;
        end for;
      end _A2V_1D_Real;

      model _V2A_1D_Real
        "Real vector to 1 dimension Real array"
        import Modelica;
        parameter Integer dims[1] = {2};
        Modelica.Blocks.Interfaces.RealInput u[product(dims)]
          annotation (Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealOutput y[dims[1]]
          annotation (Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
      protected
        Integer pos;
      algorithm
        pos := 1;
        for i1 in 1:dims[1] loop
          y[i1] := u[pos];
          pos := pos + 1;
        end for;
      end _V2A_1D_Real;
    end ArrayConverter;
  end ImportedTypes;

  model Demo
    "Bind the Syslab shaper to a UAV path-planning command source"
    QuadrotorModel.PathPlanning.EightPath rawPath
      annotation (Placement(transformation(origin = {-40, 0}, extent = {{-20, -20}, {20, 20}})));
    ImportedTypes.UAVRefShaper shaper(period = 0.02)
      annotation (Placement(transformation(origin = {35, 0}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealOutput shaped_cmd[3]
      annotation (Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
    annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 40, Tolerance = 0.0001, Interval = 0.02),
      Documentation(info = "<html><p>MWORKS platform demo: the original UAV path command is generated by QuadrotorModel.PathPlanning.EightPath and then shaped online by a Syslab function block.</p></html>"));
  equation
    connect(rawPath.position_command, shaper.in_cmd)
      annotation (Line(origin = {-2.5, 0}, points = {{-17.5, 0}, {17.5, 0}}, color = {0, 0, 127}));
    connect(shaper.out_cmd, shaped_cmd)
      annotation (Line(origin = {72.5, 0}, points = {{-17.5, 0}, {37.5, 0}}, color = {0, 0, 127}));
  end Demo;
end UAVSyslabReferenceShaper;
