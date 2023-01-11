within NHES.Systems.BalanceOfPlant.Turbine.ControlSystems;
model CS_PressureAndPowerControl_HTGRcoupled_v1
  extends BaseClasses.Partial_ControlSystem;

  parameter SI.Time delayStartTCV = 0 "Delay start of TCV control";
  parameter SI.Time delayStartBV = 0 "Delay start of BV control";

  parameter SI.Pressure p_nominal "Nominal steam turbine pressure";

  //parameter Real TCV_opening_nominal = 0.5 "Nominal opening of TCV - controls power";
  //parameter Real BV_opening_nominal = 0.001 "Nominal opening of BV - controls pressure";

  input SI.Power W_totalSetpoint "Total setpoint power from BOP" annotation(Dialog(group="Inputs"));

  TRANSFORM.Controls.LimPID
                       PID_TCV_opening(
    k=-1,
    yMax=1,
    yMin=0,
    k_s=0.25e-8,
    k_m=0.25e-8,
    controllerType=Modelica.Blocks.Types.SimpleController.PI)
           annotation (Placement(transformation(extent={{-60,-4},{-40,16}})));
  Modelica.Blocks.Sources.Constant p_Nominal(k=14000000)
    annotation (Placement(transformation(extent={{-92,-4},{-72,16}})));
  TRANSFORM.Controls.LimPID PID_BV_opening(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    yMax=1,
    yMin=0,
    k=-1,
    k_s=0.1*(1/p_nominal),
    k_m=0.1*(1/p_nominal))
    annotation (Placement(transformation(extent={{-60,-48},{-40,-28}})));
  Modelica.Blocks.Sources.RealExpression W_totalSetpoint_BOP(y=W_totalSetpoint)
    annotation (Placement(transformation(extent={{-90,-48},{-70,-28}})));
equation

  connect(actuatorBus.opening_BV, PID_BV_opening.y) annotation (Line(
      points={{30.1,-99.9},{30.1,-38},{-39,-38}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(actuatorBus.opening_TCV, PID_TCV_opening.y) annotation (Line(
      points={{30.1,-99.9},{30.1,6},{-39,6}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(p_Nominal.y, PID_TCV_opening.u_s)
    annotation (Line(points={{-71,6},{-62,6}},     color={0,0,127}));
  connect(W_totalSetpoint_BOP.y, PID_BV_opening.u_s)
    annotation (Line(points={{-69,-38},{-62,-38}},
                                                 color={0,0,127}));
  connect(sensorBus.p_inlet_steamTurbine, PID_TCV_opening.u_m) annotation (Line(
      points={{-29.9,-99.9},{-29.9,-56},{-30,-56},{-30,-14},{-50,-14},{-50,-6}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(sensorBus.W_total, PID_BV_opening.u_m) annotation (Line(
      points={{-29.9,-99.9},{-29.9,-60},{-50,-60},{-50,-50}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
annotation (defaultComponentName="EM_CS",
Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
                                                   Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-180,-100},{160,260}})));
end CS_PressureAndPowerControl_HTGRcoupled_v1;
