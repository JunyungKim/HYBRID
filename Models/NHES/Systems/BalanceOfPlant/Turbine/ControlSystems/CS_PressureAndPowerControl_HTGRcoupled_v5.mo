within NHES.Systems.BalanceOfPlant.Turbine.ControlSystems;
model CS_PressureAndPowerControl_HTGRcoupled_v5
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
  Modelica.Blocks.Sources.RealExpression W_totalSetpoint_BOP(y=W_totalSetpoint)
    annotation (Placement(transformation(extent={{-108,-4},{-88,16}})));
  Modelica.Blocks.Sources.Constant p_Nominal1(k=14000000)
    annotation (Placement(transformation(extent={{-92,50},{-72,70}})));
  TRANSFORM.Controls.LimPID
                       PID_TCV_opening1(
    k=-1,
    yMax=1,
    yMin=0,
    k_s=0.25e-8,
    k_m=0.25e-8,
    controllerType=Modelica.Blocks.Types.SimpleController.PI)
           annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  Modelica.Blocks.Sources.Constant const2(k=-1e-1)
    annotation (Placement(transformation(extent={{-170,162},{-154,178}})));
  Modelica.Blocks.Sources.Constant const10(k=5000)
    annotation (Placement(transformation(extent={{-170,136},{-154,152}})));
  Modelica.Blocks.Sources.Constant const1(k=-150)
    annotation (Placement(transformation(extent={{-170,108},{-154,124}})));
  Modelica.Blocks.Sources.Constant const3(k=14000000)
    annotation (Placement(transformation(extent={{-210,66},{-194,82}})));
  PrimaryHeatSystem.HTGR.VarLimVarK_PID PID(
    use_k_in=true,
    use_lowlim_in=true,
    use_uplim_in=true,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    with_FF=true,
    k=-5e-1,
    Ti=30) annotation (Placement(transformation(extent={{-148,74},{-128,94}})));
  Modelica.Blocks.Math.Add         add
    annotation (Placement(transformation(extent={{-94,88},{-74,108}})));
  Modelica.Blocks.Sources.Constant const4(k=1200)
    annotation (Placement(transformation(extent={{-126,106},{-110,122}})));
  Modelica.Blocks.Sources.Constant const5(k=1)
    annotation (Placement(transformation(extent={{-60,-48},{-40,-28}})));
  Modelica.Blocks.Sources.Constant constant_0(k=0)
    annotation (Placement(transformation(extent={{-210,98},{-192,116}})));
equation

  connect(actuatorBus.opening_TCV, PID_TCV_opening.y) annotation (Line(
      points={{30.1,-99.9},{30.1,6},{-39,6}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(p_Nominal1.y, PID_TCV_opening1.u_s)
    annotation (Line(points={{-71,60},{-62,60}}, color={0,0,127}));
  connect(sensorBus.p_inlet_steamTurbine, PID_TCV_opening1.u_m) annotation (
      Line(
      points={{-29.9,-99.9},{-28,-99.9},{-28,38},{-50,38},{-50,48}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(W_totalSetpoint_BOP.y, PID_TCV_opening.u_s)
    annotation (Line(points={{-87,6},{-62,6}}, color={0,0,127}));
  connect(sensorBus.W_total, PID_TCV_opening.u_m) annotation (Line(
      points={{-29.9,-99.9},{-28,-99.9},{-28,-14},{-50,-14},{-50,-6}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(const2.y,PID. prop_k) annotation (Line(points={{-153.2,170},{-132,170},
          {-132,95.4},{-130.6,95.4}},
                                 color={0,0,127}));
  connect(const10.y,PID. upperlim)
    annotation (Line(points={{-153.2,144},{-144,144},{-144,95}},
                                                             color={0,0,127}));
  connect(const1.y,PID. lowerlim) annotation (Line(points={{-153.2,116},{-138,
          116},{-138,95}},
                      color={0,0,127}));
  connect(const4.y,add. u1) annotation (Line(points={{-109.2,114},{-104,114},{
          -104,104},{-96,104}},
                           color={0,0,127}));
  connect(PID.y,add. u2) annotation (Line(points={{-127,84},{-104,84},{-104,92},
          {-96,92}},  color={0,0,127}));
  connect(const3.y,PID. u_s)
    annotation (Line(points={{-193.2,74},{-158,74},{-158,84},{-150,84}},
                                                     color={0,0,127}));
  connect(sensorBus.Steam_Pressure, PID.u_m) annotation (Line(
      points={{-30,-100},{-30,-102},{-138,-102},{-138,72}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(actuatorBus.Feed_Pump_Speed, add.y) annotation (Line(
      points={{30,-100},{30,98},{-73,98}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(actuatorBus.opening_BV, const5.y) annotation (Line(
      points={{30.1,-99.9},{30.1,-38},{-39,-38}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(constant_0.y, PID.u_ff) annotation (Line(points={{-191.1,107},{-176,
          107},{-176,92},{-150,92}}, color={0,0,127}));
annotation (defaultComponentName="EM_CS",
Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
                                                   Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-180,-100},{160,260}})));
end CS_PressureAndPowerControl_HTGRcoupled_v5;
