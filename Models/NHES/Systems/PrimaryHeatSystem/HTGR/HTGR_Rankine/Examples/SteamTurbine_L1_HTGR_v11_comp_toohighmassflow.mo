within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples;
model SteamTurbine_L1_HTGR_v11_comp_toohighmassflow
  import NHES;
  extends Modelica.Icons.Example;
  NHES.Systems.BalanceOfPlant.Turbine.SteamTurbine_L1_boundaries_type2 BOP(
    nPorts_a3=1,
    port_a3_nominal_m_flow={0},
    port_a_nominal(
      m_flow=45.7058,
      p=14000000,
      h=BOP.Medium.specificEnthalpy_pT(BOP.port_a_nominal.p, 813)),
    port_b_nominal(p=14000000, h=BOP.Medium.specificEnthalpy_pT(BOP.port_b_nominal.p,
          483)),
    redeclare
      NHES.Systems.BalanceOfPlant.Turbine.ControlSystems.CS_PressureAndPowerControl
      CS(
      delayStartTCV=0,
      delayStartBV=0,
      p_nominal=14000000,
      W_totalSetpoint=const.y),
    reservoir(level_start=1400))
    annotation (Placement(transformation(extent={{-30,-30},{30,30}})));
  TRANSFORM.Electrical.Sources.FrequencySource
                                     sinkElec(f=60)
    annotation (Placement(transformation(extent={{90,-10},{70,10}})));
  Fluid.Sensors.stateSensor stateSensor(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-44,-22},{-64,-2}})));
  Fluid.Sensors.stateSensor stateSensor1(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-64,2},{-44,22}})));
  Fluid.Sensors.stateDisplay stateDisplay
    annotation (Placement(transformation(extent={{-76,-60},{-32,-30}})));
  Fluid.Sensors.stateDisplay stateDisplay1
    annotation (Placement(transformation(extent={{-76,20},{-32,50}})));
  Modelica.Fluid.Sources.MassFlowSource_h source1(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    nPorts=1,
    use_m_flow_in=false,
    m_flow=10,
    h=3e6)
    annotation (Placement(transformation(extent={{-40,-90},{-20,-70}})));
  Modelica.Blocks.Sources.Sine sine(
    f=1/200,
    offset=4e7,
    startTime=1500,
    amplitude=2e8)
    annotation (Placement(transformation(extent={{-96,76},{-76,96}})));
  NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Components.HTGR_PebbleBed_Primary_Loop_STHX
                                              hTGR_PebbleBed_Primary_Loop(
      redeclare
      NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.ControlSystems.CS_Rankine_Primary_SS
      CS) annotation (Placement(transformation(extent={{-134,-30},{-76,26}})));
  Modelica.Blocks.Sources.Constant
                               const(k=4e7)
    annotation (Placement(transformation(extent={{-68,76},{-48,96}})));
equation
  hTGR_PebbleBed_Primary_Loop.input_steam_pressure =
    BOP.pressure.p;

  connect(stateDisplay1.statePort, stateSensor1.statePort) annotation (Line(
        points={{-54,31.1},{-54,12.05},{-53.95,12.05}},            color={0,0,0}));
  connect(stateDisplay.statePort, stateSensor.statePort) annotation (Line(
        points={{-54,-48.9},{-54,-11.95},{-54.05,-11.95}}, color={0,0,0}));
  connect(stateSensor.port_a, BOP.port_b)
    annotation (Line(points={{-44,-12},{-30,-12}}, color={0,127,255}));
  connect(stateSensor1.port_b, BOP.port_a)
    annotation (Line(points={{-44,12},{-30,12}}, color={0,127,255}));
  connect(source1.ports[1], BOP.port_a3[1]) annotation (Line(points={{-20,-80},
          {-12,-80},{-12,-30}}, color={0,127,255}));
  connect(BOP.portElec_b, sinkElec.port)
    annotation (Line(points={{30,0},{70,0}}, color={255,0,0}));
  connect(hTGR_PebbleBed_Primary_Loop.port_b, stateSensor1.port_a) annotation (
      Line(points={{-76.87,11.72},{-76.87,12},{-64,12}}, color={0,127,255}));
  connect(stateSensor.port_b, hTGR_PebbleBed_Primary_Loop.port_a) annotation (
      Line(points={{-64,-12},{-76.87,-12},{-76.87,-11.24}}, color={0,127,255}));
  annotation (experiment(
      StartTime=300,
      StopTime=20000,
      __Dymola_NumberOfIntervals=200,
      __Dymola_Algorithm="Esdirk34a"));
end SteamTurbine_L1_HTGR_v11_comp_toohighmassflow;
