within NHES.Systems.EnergyStorage.SHS_Two_Tank.ControlSystems;
model CS_TES_VN3

  extends
    NHES.Systems.EnergyStorage.SHS_Two_Tank.BaseClasses.Partial_ControlSystem;

  NHES.Systems.EnergyStorage.SHS_Two_Tank.Data.Data_Default data
    annotation (Placement(transformation(extent={{-50,136},{-30,156}})));
  Modelica.Blocks.Sources.Trapezoid trapezoid(
    amplitude=100.0,
    rising=500,
    width=850,
    falling=500,
    period=3300,
    offset=0.0,
    startTime=0)
    annotation (Placement(transformation(extent={{-132,88},{-120,100}})));
  NHES.Systems.BalanceOfPlant.StagebyStageTurbineSecondary.Control_and_Distribution.MinMaxFilter
    Discharging_Valve_Position(min=1e-4) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={12,64})));
  Modelica.Blocks.Sources.Constant one2(k=20)
    annotation (Placement(transformation(extent={{-116,74},{-110,80}})));
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
  TRANSFORM.Controls.LimPID PID1(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=5e-3,
    Ti=200,
    yMax=1.0,
    yMin=0.0,
    initType=Modelica.Blocks.Types.Init.InitialOutput,
    y_start=0.2)
    annotation (Placement(transformation(extent={{-32,94},{-24,102}})));
  Modelica.Blocks.Math.Product product2
    annotation (Placement(transformation(extent={{-16,96},{-8,104}})));
  Modelica.Blocks.Math.Add add2
    annotation (Placement(transformation(extent={{-48,102},{-40,110}})));
  Modelica.Blocks.Math.Min min2
    annotation (Placement(transformation(extent={{-68,90},{-60,98}})));
  Modelica.Blocks.Sources.Constant one5(k=-0.25)
    annotation (Placement(transformation(extent={{-66,104},{-60,110}})));
  Modelica.Blocks.Sources.Constant one6(k=0.5)
    annotation (Placement(transformation(extent={{-82,92},{-76,98}})));
  Modelica.Blocks.Sources.Constant one8(k=273.15 + 230)
    annotation (Placement(transformation(extent={{-44,94},{-38,100}})));
  Modelica.Blocks.Math.Gain gain1(k=2.5)
    annotation (Placement(transformation(extent={{-56,92},{-52,96}})));
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
  connect(product2.y, Discharging_Valve_Position.u)
    annotation (Line(points={{-7.6,100},{-2,100},{-2,72},{-8,72},{-8,64},{0,
          64}},                                  color={0,0,127}));
  connect(PID1.y,product2. u2) annotation (Line(points={{-23.6,98},{-20.2,
          98},{-20.2,97.6},{-16.8,97.6}},                    color={0,0,127}));
  connect(add2.y,product2. u1) annotation (Line(points={{-39.6,106},{-39.6,
          102.4},{-16.8,102.4}},                       color={0,0,127}));
  connect(one5.y,add2. u1) annotation (Line(points={{-59.7,107},{-54.25,107},
          {-54.25,108.4},{-48.8,108.4}},                          color={0,0,
          127}));
  connect(sensorBus.hot_tank_level,min2. u1) annotation (Line(
      points={{-30,-100},{-30,-30},{-70,-30},{-70,86},{-72,86},{-72,96.4},{
          -68.8,96.4}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(one8.y,PID1. u_s) annotation (Line(points={{-37.7,97},{-35.25,97},
          {-35.25,98},{-32.8,98}},
                                 color={0,0,127}));
  connect(add2.u2,gain1. y) annotation (Line(points={{-48.8,103.6},{-51.8,
          103.6},{-51.8,94}}, color={0,0,127}));
  connect(sensorBus.Cold_Tank_Entrance_Temp, PID1.u_m) annotation (Line(
      points={{-30,-100},{-92,-100},{-92,78},{-28,78},{-28,93.2}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(min2.u2, one6.y) annotation (Line(points={{-68.8,91.6},{-75.7,
          91.6},{-75.7,95}}, color={0,0,127}));
  connect(gain1.u, min2.y)
    annotation (Line(points={{-56.4,94},{-59.6,94}}, color={0,0,127}));
annotation(defaultComponentName="changeMe_CS", Icon(graphics={
        Text(
          extent={{-94,82},{94,74}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={255,255,237},
          fillPattern=FillPattern.Solid,
          textString="Change Me")}));
end CS_TES_VN3;
