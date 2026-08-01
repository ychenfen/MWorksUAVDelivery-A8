within;
package QuadrotorTask
  extends Modelica.Icons.Package;

  model ClimbBaseline
    extends QuadrotorModel.Examples.Example1;
    annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 50, Tolerance = 0.0001, Interval = 0.01));
  end ClimbBaseline;

  model ClimbEnhanced
    extends QuadrotorModel.Examples.Example1(
      redeclare QuadrotorModel.Blocks.Controller.SlidingModeController controller3_2);
    annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 50, Tolerance = 0.0001, Interval = 0.01));
  end ClimbEnhanced;

  model SpiralBaseline
    extends QuadrotorModel.Examples.Example2;
    annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 50, Tolerance = 0.0001, Interval = 0.01));
  end SpiralBaseline;

  model SpiralEnhanced
    extends QuadrotorModel.Examples.Example2(
      redeclare QuadrotorModel.Blocks.Controller.SlidingModeController controller3_2);
    annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 50, Tolerance = 0.0001, Interval = 0.01));
  end SpiralEnhanced;

  model EightBaseline
    extends QuadrotorModel.Examples.Example3;
    annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 120, Tolerance = 0.0001, Interval = 0.01));
  end EightBaseline;

  model EightEnhanced
    extends QuadrotorModel.Examples.Example3(
      redeclare QuadrotorModel.Blocks.Controller.SlidingModeController controller3_2);
    annotation (experiment(Algorithm = Dassl, StartTime = 0, StopTime = 120, Tolerance = 0.0001, Interval = 0.01));
  end EightEnhanced;
end QuadrotorTask;
