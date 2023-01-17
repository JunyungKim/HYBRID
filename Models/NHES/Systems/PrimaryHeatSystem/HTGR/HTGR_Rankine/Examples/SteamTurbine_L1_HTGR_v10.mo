within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples;
model SteamTurbine_L1_HTGR_v10
  import NHES;
  extends Modelica.Icons.Example;
  NHES.Systems.BalanceOfPlant.Turbine.SteamTurbine_L1_boundaries_type4 BOP(
    nPorts_a3=1,
    port_a3_nominal_m_flow={10},
    port_a_nominal(
      m_flow=493.7058,
      p=5550000,
      h=BOP.Medium.specificEnthalpy_pT(BOP.port_a_nominal.p, 591)),
    port_b_nominal(p=1000000, h=BOP.Medium.specificEnthalpy_pT(BOP.port_b_nominal.p,
          318.95)),
    redeclare
      NHES.Systems.BalanceOfPlant.Turbine.ControlSystems.CS_PressureAndPowerControl_HTGRcoupled_v5
      CS(p_nominal=14000000, W_totalSetpoint=sine.y),
    reservoir(level_start=1400))
    annotation (Placement(transformation(extent={{0,-30},{60,30}})));
  TRANSFORM.Electrical.Sources.FrequencySource
                                     sinkElec(f=60)
    annotation (Placement(transformation(extent={{90,-10},{70,10}})));
  Fluid.Sensors.stateSensor stateSensor(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-8,-22},{-28,-2}})));
  Fluid.Sensors.stateSensor stateSensor1(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-28,2},{-8,22}})));
  Fluid.Sensors.stateDisplay stateDisplay
    annotation (Placement(transformation(extent={{-40,-74},{4,-44}})));
  Fluid.Sensors.stateDisplay stateDisplay1
    annotation (Placement(transformation(extent={{-40,20},{4,50}})));
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
    annotation (Placement(transformation(extent={{-70,70},{-50,90}})));
  NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Components.HTGR_PebbleBed_Primary_Loop_STHX
                                              hTGR_PebbleBed_Primary_Loop(
      redeclare
      NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.ControlSystems.CS_Rankine_Primary_SS
      CS) annotation (Placement(transformation(extent={{-96,-30},{-38,26}})));
equation
  hTGR_PebbleBed_Primary_Loop.input_steam_pressure = BOP.pressure.p;

  connect(stateDisplay1.statePort, stateSensor1.statePort) annotation (Line(
        points={{-18,31.1},{-18,12.05},{-17.95,12.05}},            color={0,0,0}));
  connect(stateDisplay.statePort, stateSensor.statePort) annotation (Line(
        points={{-18,-62.9},{-18,-11.95},{-18.05,-11.95}}, color={0,0,0}));
  connect(stateSensor1.port_b, BOP.port_a)
    annotation (Line(points={{-8,12},{0,12}},    color={0,127,255}));
  connect(source1.ports[1], BOP.port_a3[1]) annotation (Line(points={{-20,-80},{
          18,-80},{18,-30}},    color={0,127,255}));
  connect(BOP.portElec_b, sinkElec.port)
    annotation (Line(points={{60,0},{70,0}}, color={255,0,0}));
  connect(hTGR_PebbleBed_Primary_Loop.port_b, stateSensor1.port_a) annotation (
      Line(points={{-38.87,11.72},{-38.87,12},{-28,12}}, color={0,127,255}));
  connect(stateSensor.port_b, hTGR_PebbleBed_Primary_Loop.port_a) annotation (
      Line(points={{-28,-12},{-38.87,-12},{-38.87,-11.24}}, color={0,127,255}));
  connect(stateSensor.port_a, BOP.port_b)
    annotation (Line(points={{-8,-12},{0,-12}}, color={0,127,255}));
  annotation (experiment(StopTime=200, __Dymola_Algorithm="Dassl"));
end SteamTurbine_L1_HTGR_v10;
