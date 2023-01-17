within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples;
model SteamTurbine_L1_HTGR_v12_comp_lowSteamTemp
  import NHES;
  extends Modelica.Icons.Example;
  NHES.Systems.BalanceOfPlant.Turbine.SteamTurbine_L1_boundaries_type1 BOP(
    nPorts_a3=1,
    port_a3_nominal_m_flow={0},
    port_a_nominal(
      m_flow=45.7058,
      p=14000000,
      h=BOP.Medium.specificEnthalpy_pT(BOP.port_a_nominal.p, 813)),
    port_b_nominal(p=14000000, h=BOP.Medium.specificEnthalpy_pT(BOP.port_b_nominal.p,
          481)),
    redeclare
      NHES.Systems.BalanceOfPlant.Turbine.ControlSystems.CS_PressureAndPowerControl
      CS(
      delayStartTCV=0,
      delayStartBV=0,
      p_nominal=14000000,
      W_totalSetpoint=const.y),
    reservoir(level_start=10))
    annotation (Placement(transformation(extent={{-30,-30},{30,30}})));
  TRANSFORM.Electrical.Sources.FrequencySource
                                     sinkElec(f=60)
    annotation (Placement(transformation(extent={{90,-10},{70,10}})));
  Fluid.Sensors.stateSensor stateSensor(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-70,-22},{-90,-2}})));
  Fluid.Sensors.stateSensor stateSensor1(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-90,2},{-70,22}})));
  Fluid.Sensors.stateDisplay stateDisplay
    annotation (Placement(transformation(extent={{-102,-70},{-58,-40}})));
  Fluid.Sensors.stateDisplay stateDisplay1
    annotation (Placement(transformation(extent={{-102,20},{-58,50}})));
  Modelica.Fluid.Sources.MassFlowSource_h source1(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    nPorts=1,
    use_m_flow_in=false,
    m_flow=10,
    h=3e6)
    annotation (Placement(transformation(extent={{12,-56},{-8,-36}})));
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
      CS) annotation (Placement(transformation(extent={{-156,-30},{-98,26}})));
  Modelica.Blocks.Sources.Constant
                               const(k=4e7)
    annotation (Placement(transformation(extent={{-68,76},{-48,96}})));
  TRANSFORM.Fluid.Machines.Pump_SimpleMassFlow pump(redeclare package Medium =
        Modelica.Media.Water.StandardWater, use_input=true)
    annotation (Placement(transformation(extent={{-38,-2},{-58,-22}})));
  Modelica.Blocks.Sources.Constant
                               const1(k=35)
    annotation (Placement(transformation(extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-48,-34})));
equation
  hTGR_PebbleBed_Primary_Loop.input_steam_pressure =
    BOP.pressure.p;

  connect(stateDisplay1.statePort, stateSensor1.statePort) annotation (Line(
        points={{-80,31.1},{-80,12.05},{-79.95,12.05}},            color={0,0,0}));
  connect(stateDisplay.statePort, stateSensor.statePort) annotation (Line(
        points={{-80,-58.9},{-80,-11.95},{-80.05,-11.95}}, color={0,0,0}));
  connect(stateSensor1.port_b, BOP.port_a)
    annotation (Line(points={{-70,12},{-30,12}}, color={0,127,255}));
  connect(source1.ports[1], BOP.port_a3[1]) annotation (Line(points={{-8,-46},{
          -12,-46},{-12,-30}},  color={0,127,255}));
  connect(BOP.portElec_b, sinkElec.port)
    annotation (Line(points={{30,0},{70,0}}, color={255,0,0}));
  connect(hTGR_PebbleBed_Primary_Loop.port_b, stateSensor1.port_a) annotation (
      Line(points={{-98.87,11.72},{-98.87,12},{-90,12}}, color={0,127,255}));
  connect(stateSensor.port_b, hTGR_PebbleBed_Primary_Loop.port_a) annotation (
      Line(points={{-90,-12},{-98.87,-12},{-98.87,-11.24}}, color={0,127,255}));
  connect(BOP.port_b, pump.port_a)
    annotation (Line(points={{-30,-12},{-38,-12}}, color={0,127,255}));
  connect(pump.port_b, stateSensor.port_a)
    annotation (Line(points={{-58,-12},{-70,-12}}, color={0,127,255}));
  connect(const1.y, pump.in_m_flow)
    annotation (Line(points={{-48,-27.4},{-48,-19.3}}, color={0,0,127}));
  annotation (experiment(
      StopTime=20000,
      __Dymola_NumberOfIntervals=200,
      __Dymola_Algorithm="Esdirk34a"));
end SteamTurbine_L1_HTGR_v12_comp_lowSteamTemp;
