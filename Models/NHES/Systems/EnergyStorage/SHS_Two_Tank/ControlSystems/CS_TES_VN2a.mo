within NHES.Systems.EnergyStorage.SHS_Two_Tank.ControlSystems;
model CS_TES_VN2a

  extends
    NHES.Systems.EnergyStorage.SHS_Two_Tank.BaseClasses.Partial_ControlSystem;

  NHES.Systems.EnergyStorage.SHS_Two_Tank.Data.Data_Default data
    annotation (Placement(transformation(extent={{-50,136},{-30,156}})));
  TRANSFORM.Controls.LimPID PID3(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=2e-2,
    Ti=10,
    yMax=1.0,
    yMin=0.0,
    y_start=0.0)
    annotation (Placement(transformation(extent={{-36,54},{-28,62}})));
  Modelica.Blocks.Sources.Trapezoid trapezoid(
    amplitude=100.0,
    rising=500,
    width=850,
    falling=500,
    period=3300,
    offset=0.0,
    startTime=0)
    annotation (Placement(transformation(extent={{-110,48},{-98,60}})));
  NHES.Systems.BalanceOfPlant.StagebyStageTurbineSecondary.Control_and_Distribution.MinMaxFilter
    Discharging_Valve_Position(min=1e-4) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={12,64})));
  Modelica.Blocks.Sources.Constant one2(k=20)
    annotation (Placement(transformation(extent={{-88,36},{-82,42}})));
  TRANSFORM.Controls.LimPID PID2(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=-2.5e-2,
    Ti=10,
    yMax=1.0,
    yMin=0.0,
    y_start=0.0)
    annotation (Placement(transformation(extent={{-26,-10},{-20,-4}})));
  Modelica.Blocks.Sources.Constant one1(k=500 + 273.15)
    annotation (Placement(transformation(extent={{-60,-6},{-54,0}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=-50,
    duration=500,
    offset=200,
    startTime=5000)
    annotation (Placement(transformation(extent={{-72,34},{-60,46}})));
equation

  connect(actuatorBus.Discharge_Valve_Position, Discharging_Valve_Position.y)
    annotation (Line(
      points={{30,-100},{30,64},{23.4,64}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(one1.y, PID2.u_s)
    annotation (Line(points={{-53.7,-3},{-53.7,-4},{-30,-4},{-30,-7},{-26.6,
          -7}},                                      color={0,0,127}));
  connect(PID3.y, Discharging_Valve_Position.u) annotation (Line(points={{
          -27.6,58},{-10,58},{-10,64},{0,64}}, color={0,0,127}));
  connect(sensorBus.discharge_m_flow, PID3.u_m) annotation (Line(
      points={{-30,-100},{-36,-100},{-36,53.2},{-32,53.2}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(sensorBus.Charge_Temp, PID2.u_m) annotation (Line(
      points={{-30,-100},{-30,-10.6},{-23,-10.6}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(actuatorBus.Charge_Valve_Position, PID2.y) annotation (Line(
      points={{30,-100},{30,-7},{-19.7,-7}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(PID3.u_s, trapezoid.y) annotation (Line(points={{-36.8,58},{-64,
          58},{-64,54},{-97.4,54}}, color={0,0,127}));
annotation(defaultComponentName="changeMe_CS", Icon(graphics={
        Text(
          extent={{-94,82},{94,74}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={255,255,237},
          fillPattern=FillPattern.Solid,
          textString="Change Me")}));
end CS_TES_VN2a;
