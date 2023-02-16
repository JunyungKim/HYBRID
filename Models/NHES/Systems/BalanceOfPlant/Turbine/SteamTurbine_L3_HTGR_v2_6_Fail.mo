within NHES.Systems.BalanceOfPlant.Turbine;
model SteamTurbine_L3_HTGR_v2_6_Fail
  extends BaseClasses.Partial_SubSystem(
    redeclare replaceable
      ControlSystems.CS_threeStagedTurbine_HTGR_v2
      CS,
    redeclare replaceable ControlSystems.ED_Dummy ED,
    redeclare Data.HTGR_3_BOP   data);
  Real time_altered;
  Real time_initialization = 7e4;
  Real electricity_generation_Norm;
  Real electricity_demand_Norm;

  TRANSFORM.Fluid.Machines.SteamTurbine HPT(
    nUnits=1,
    eta_mech=0.93,
    redeclare model Eta_wetSteam =
        TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
    p_a_start=dataInitial.HPT_P_inlet,
    p_b_start=dataInitial.HPT_P_outlet,
    T_a_start=dataInitial.HPT_T_inlet,
    T_b_start=dataInitial.HPT_T_outlet,
    m_flow_nominal=data.HPT_nominal_mflow,
    p_inlet_nominal=data.HPT_p_in_nominal,
    p_outlet_nominal=data.HPT_p_exit_nominal,
    T_nominal=data.HPT_T_in_nominal)
    annotation (Placement(transformation(extent={{34,24},{54,44}})));

  TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={238,66})));
  Fluid.Vessels.IdealCondenser Condenser(
    p=data.p_condensor,
    V_total=data.V_condensor,
    V_liquid_start=dataInitial.V_condensor_liquid_start)
    annotation (Placement(transformation(extent={{244,-88},{224,-68}})));
  TRANSFORM.Fluid.Machines.Pump_Controlled FWCP(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    N_nominal=data.pump_feedWaterControl_RPM,
    dp_nominal=data.pump_feedWaterControl_dp_nominal,
    m_flow_nominal=data.pump_feedWaterControl_nominal_mflow,
    d_nominal=data.pump_feedWaterControl_nominal_density,
    controlType="RPM",
    use_port=true)
    annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                       sensor_T1(redeclare package Medium =
        Modelica.Media.Water.StandardWater)            annotation (Placement(
        transformation(
        extent={{6,6},{-6,-6}},
        rotation=180,
        origin={18,40})));
  TRANSFORM.Fluid.Sensors.Pressure     sensor_p(redeclare package Medium =
        Modelica.Media.Water.StandardWater, redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar)
                                                       annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-14,76})));
  TRANSFORM.Fluid.Volumes.SimpleVolume volume(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    p_start=3900000,
    T_start=723.15,
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
        (V=2),
    use_TraceMassPort=false)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-62,48})));

  TRANSFORM.Fluid.Valves.ValveLinear TCV(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    dp_nominal=data.valve_TCV_dp_nominal,
    m_flow_nominal=data.valve_TCV_mflow)
                       annotation (Placement(transformation(
        extent={{8,8},{-8,-8}},
        rotation=180,
        origin={-4,40})));
  Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
    annotation (Placement(transformation(extent={{-106,108},{-86,116}})));
  TRANSFORM.Fluid.Machines.SteamTurbine LPT1(
    nUnits=1,
    eta_mech=0.93,
    redeclare model Eta_wetSteam =
        TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
    p_a_start=dataInitial.LPT1_P_inlet,
    p_b_start=dataInitial.LPT1_P_outlet,
    T_a_start=dataInitial.LPT1_T_inlet,
    T_b_start=dataInitial.LPT1_T_outlet,
    m_flow_nominal=data.LPT1_nominal_mflow,
    p_inlet_nominal=data.LPT1_p_in_nominal,
    p_outlet_nominal=data.LPT1_p_exit_nominal,
    T_nominal=data.LPT1_T_in_nominal) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={128,34})));

  TRANSFORM.Fluid.Volumes.SimpleVolume volume1(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    p_start=3900000,
    T_start=473.15,
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
        (V=5.0),
    use_TraceMassPort=false)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={78,-58})));
  TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    V=data.V_tee1,
    p_start=2500000,
    T_start=573.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={72,50})));
  TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    dp_nominal=data.valve_LPT1_Bypass_dp_nominal,
    m_flow_nominal=data.valve_LPT1_Bypass_mflow)
                      annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={72,16})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                       sensor_T2(redeclare package Medium =
        Modelica.Media.Water.StandardWater)            annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={-108,-58})));
  TRANSFORM.Fluid.Machines.Pump_PressureBooster FWP(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    use_input=false,
    p_nominal=data.pump_feedWater_nominal_pressure,
    allowFlowReversal=false)
    annotation (Placement(transformation(extent={{190,-108},{170,-88}})));
  TRANSFORM.Fluid.Valves.ValveLinear TBV(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    dp_nominal=data.valve_TBV_dp_nominal,
    m_flow_nominal=data.valve_TBV_mflow)
                       annotation (Placement(transformation(
        extent={{-8,8},{8,-8}},
        rotation=180,
        origin={-74,72})));
  TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    p=12000000,
    T=573.15,
    nPorts=1)
    annotation (Placement(transformation(extent={{-150,62},{-130,82}})));
  TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-150,38},{-130,58}})));
  TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-150,-68},{-130,-48}})));
  TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
    annotation (Placement(transformation(extent={{130,-10},{150,10}}),
        iconTransformation(extent={{130,-10},{150,10}})));
  TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee2(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    V=data.V_tee2,
    p_start=1500000,
    T_start=423.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={178,50})));
  TRANSFORM.Fluid.Machines.SteamTurbine LPT2(
    nUnits=1,
    eta_mech=0.93,
    redeclare model Eta_wetSteam =
        TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
    p_a_start=dataInitial.LPT2_P_inlet,
    p_b_start=dataInitial.LPT2_P_outlet,
    T_a_start=dataInitial.LPT2_T_inlet,
    T_b_start=dataInitial.LPT2_T_outlet,
    m_flow_nominal=data.LPT2_nominal_mflow,
    p_inlet_nominal=data.LPT2_p_in_nominal,
    p_outlet_nominal=data.LPT2_p_exit_nominal,
    T_nominal=data.LPT2_T_in_nominal) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={208,34})));

  TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    dp_nominal=data.valve_LPT2_Bypass_dp_nominal,
    m_flow_nominal=data.valve_LPT2_Bypass_mflow)
                      annotation (Placement(transformation(
        extent={{6,6},{-6,-6}},
        rotation=90,
        origin={178,26})));
  Data.HTGR_3_BOP_Init dataInitial(LPT2_T_inlet=473.15)
    annotation (Placement(transformation(extent={{66,120},{86,140}})));
  StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
    Moisture_Separator2(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    p_start=2500000,
    T_start=573.15,
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
    annotation (Placement(transformation(extent={{140,40},{160,60}})));
  TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium
      = Modelica.Media.Water.StandardWater)            annotation (Placement(
        transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={108,0})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                       sensor_T3(redeclare package Medium =
        Modelica.Media.Water.StandardWater)            annotation (Placement(
        transformation(
        extent={{6,6},{-6,-6}},
        rotation=180,
        origin={86,0})));

  Fluid.HeatExchangers.Generic_HXs.NTU_HX_SinglePhase BypassFeedwaterHeater(
    NTU=data.BypassFeedHeater_NTU,
    K_tube=data.BypassFeedHeater_K_tube,
    K_shell=data.BypassFeedHeater_K_shell,
    redeclare package Tube_medium = Modelica.Media.Water.StandardWater,
    redeclare package Shell_medium = Modelica.Media.Water.StandardWater,
    V_Tube=data.BypassFeedHeater_V_tube,
    V_Shell=data.BypassFeedHeater_V_shell,
    p_start_tube=dataInitial.BypassFeedHeater_tube_p_start,
    use_T_start_tube=true,
    T_start_tube_inlet=dataInitial.BypassFeedHeater_tube_T_start_inlet,
    T_start_tube_outlet=dataInitial.BypassFeedHeater_tube_T_start_outlet,
    h_start_tube_inlet=dataInitial.BypassFeedHeater_h_start_tube_inlet,
    h_start_tube_outlet=dataInitial.BypassFeedHeater_h_start_tube_outlet,
    p_start_shell=dataInitial.BypassFeedHeater_shell_p_start,
    use_T_start_shell=true,
    T_start_shell_inlet=dataInitial.BypassFeedHeater_shell_T_start_inlet,
    T_start_shell_outlet=dataInitial.BypassFeedHeater_shell_T_start_outlet,
    h_start_shell_inlet=dataInitial.BypassFeedHeater_h_start_shell_inlet,
    h_start_shell_outlet=dataInitial.BypassFeedHeater_h_start_shell_outlet,
    dp_init_tube=dataInitial.BypassFeedHeater_dp_init_tube,
    dp_init_shell=dataInitial.BypassFeedHeater_dp_init_shell,
    Q_init=dataInitial.BypassFeedHeater_Q_init,
    m_start_tube=dataInitial.BypassFeedHeater_m_start_tube,
    m_start_shell=dataInitial.BypassFeedHeater_m_start_shell)
    annotation (Placement(transformation(extent={{8,-44},{28,-64}})));
  TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass1(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    dp_nominal=1000000,
    m_flow_nominal=0.1)
                      annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={162,-52})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=1)
    annotation (Placement(transformation(extent={{194,-46},{174,-26}})));
  Fluid.HeatExchangers.Generic_HXs.NTU_HX_SinglePhase BypassFeedwaterHeater1(
    NTU=data.BypassFeedHeater_NTU,
    K_tube=data.BypassFeedHeater_K_tube,
    K_shell=data.BypassFeedHeater_K_shell,
    redeclare package Tube_medium = Modelica.Media.Water.StandardWater,
    redeclare package Shell_medium = Modelica.Media.Water.StandardWater,
    V_Tube=data.BypassFeedHeater_V_tube,
    V_Shell=data.BypassFeedHeater_V_shell,
    p_start_tube=dataInitial.BypassFeedHeater_tube_p_start,
    use_T_start_tube=true,
    T_start_tube_inlet=dataInitial.BypassFeedHeater_tube_T_start_inlet,
    T_start_tube_outlet=dataInitial.BypassFeedHeater_tube_T_start_outlet,
    h_start_tube_inlet=dataInitial.BypassFeedHeater_h_start_tube_inlet,
    h_start_tube_outlet=dataInitial.BypassFeedHeater_h_start_tube_outlet,
    p_start_shell=dataInitial.BypassFeedHeater_shell_p_start,
    use_T_start_shell=true,
    T_start_shell_inlet=dataInitial.BypassFeedHeater_shell_T_start_inlet,
    T_start_shell_outlet=dataInitial.BypassFeedHeater_shell_T_start_outlet,
    h_start_shell_inlet=dataInitial.BypassFeedHeater_h_start_shell_inlet,
    h_start_shell_outlet=dataInitial.BypassFeedHeater_h_start_shell_outlet,
    dp_init_tube=dataInitial.BypassFeedHeater_dp_init_tube,
    dp_init_shell=dataInitial.BypassFeedHeater_dp_init_shell,
    Q_init=dataInitial.BypassFeedHeater_Q_init,
    m_start_tube=dataInitial.BypassFeedHeater_m_start_tube,
    m_start_shell=dataInitial.BypassFeedHeater_m_start_shell)
    annotation (Placement(transformation(extent={{154,8},{174,-12}})));
  Modelica.Fluid.Sources.MassFlowSource_T source(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    m_flow=50,
    T=318.15,
    nPorts=1)
    annotation (Placement(transformation(extent={{292,-16},{272,4}})));
  Modelica.Fluid.Sources.Boundary_ph sink(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    h=2000000,
    nPorts=1,
    p(displayUnit="bar") = 10000)
    annotation (Placement(transformation(extent={{-32,4},{-12,-16}})));
initial equation

equation
  port_e.W = generator.power;
  time_altered = time-time_initialization;
  electricity_generation_Norm = generator.power/44E+6;
  electricity_demand_Norm     = CS.trap_LTV1bypass_power.y/44E+6;

  connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
      points={{34,40},{24,40}},
      color={0,127,255},
      thickness=0.5));
  connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
      points={{-30,100},{4,100},{4,42.16},{18,42.16}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorBus.Feed_Pump_Speed,FWCP. inputSignal) annotation (Line(
      points={{30,100},{254,100},{254,-104},{-34,-104},{-34,-65}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
      points={{-30,100},{-30,76},{-20,76}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
      points={{4,40},{12,40}},
      color={0,127,255},
      thickness=0.5));
  connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
      points={{-30,100},{-30,76},{-80,76},{-80,112},{-85,112}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(LPT1.portHP, tee1.port_1) annotation (Line(
      points={{118,40},{118,50},{82,50}},
      color={0,127,255},
      thickness=0.5));
  connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
      points={{72,40},{72,26}},
      color={0,127,255},
      thickness=0.5));
  connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
      points={{-30,100},{-30,88},{-108,88},{-108,-54.4}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(Condenser.port_b, FWP.port_a) annotation (Line(
      points={{234,-88},{234,-98},{190,-98}},
      color={0,127,255},
      thickness=0.5));
  connect(FWP.port_b, volume1.port_a) annotation (Line(
      points={{170,-98},{84,-98},{84,-58}},
      color={0,127,255},
      thickness=0.5));
  connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
      points={{54,34},{118,34}},
      color={0,0,0},
      pattern=LinePattern.Dash));

  connect(TBV.port_b, boundary1.ports[1])
    annotation (Line(points={{-82,72},{-130,72}},color={0,127,255}));
  connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
      points={{30.1,100.1},{-4,100.1},{-4,46.4}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,48},{-46,
          48},{-46,72},{-66,72}},
                              color={0,127,255}));
  connect(volume.port_b, TCV.port_a)
    annotation (Line(points={{-56,48},{-34,48},{-34,40},{-12,40}},
                                                 color={0,127,255}));
  connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,48},{
          -34,48},{-34,62},{-14,62},{-14,66}},
                                       color={0,127,255}));
  connect(sensor_T2.port_b, port_b)
    annotation (Line(points={{-118,-58},{-140,-58}},color={0,127,255}));
  connect(TBV.opening, actuatorBus.TBV) annotation (Line(points={{-74,78.4},{-74,
          100},{30,100}},       color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(LPT1.shaft_b, LPT2.shaft_a)
    annotation (Line(points={{138,34},{198,34}}, color={0,0,0}));
  connect(tee2.port_1, LPT2.portHP) annotation (Line(points={{188,50},{192,50},{
          192,40},{198,40}}, color={0,127,255}));
  connect(tee2.port_3, LPT2_Bypass.port_a)
    annotation (Line(points={{178,40},{178,32}},color={0,127,255}));
  connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
      Line(
      points={{30,100},{248,100},{248,26},{182.8,26}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
          138,40},{138,50},{144,50}}, color={0,127,255}));
  connect(Moisture_Separator2.port_b, tee2.port_2)
    annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
  connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
        points={{146,46},{146,-58},{84,-58}},                    color={0,127,
          255}));
  connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{62,40},{62,
          50}},            color={0,127,255}));
  connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
      points={{-30,100},{-30,88},{108,88},{108,3.6}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(actuatorBus.openingLPTv, LPT1_Bypass.opening) annotation (Line(
      points={{30,100},{110,100},{110,16},{80,16}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(generator.shaft, LPT2.shaft_b) annotation (Line(points={{238.1,55.9},
          {238.1,34},{218,34}}, color={0,0,0}));
  connect(sensor_T2.port_a,FWCP. port_b)
    annotation (Line(points={{-98,-58},{-44,-58}}, color={0,127,255}));
  connect(LPT1_Bypass.port_b, sensor_T3.port_a) annotation (Line(points={{72,6},{
          72,-8.88178e-16},{80,-8.88178e-16}},
                                   color={0,127,255}));
  connect(sensor_T3.port_b, sensor_m_flow.port_a)
    annotation (Line(points={{92,4.44089e-16},{92,-1.77636e-15},{98,
          -1.77636e-15}},                        color={0,127,255}));
  connect(port_a, volume.port_a)
    annotation (Line(points={{-140,48},{-68,48}}, color={0,127,255}));
  connect(volume1.port_b, BypassFeedwaterHeater.Tube_in)
    annotation (Line(points={{72,-58},{28,-58}}, color={0,127,255}));
  connect(BypassFeedwaterHeater.Tube_out, FWCP.port_a)
    annotation (Line(points={{8,-58},{-24,-58}}, color={0,127,255}));
  connect(LPT2_Bypass.port_b, BypassFeedwaterHeater.Shell_in) annotation (Line(
        points={{178,20},{178,-16},{4,-16},{4,-52},{8,-52}},color={0,127,255}));
  connect(BypassFeedwaterHeater.Shell_out, LPT2_Bypass1.port_a)
    annotation (Line(points={{28,-52},{152,-52}}, color={0,127,255}));
  connect(realExpression.y, LPT2_Bypass1.opening)
    annotation (Line(points={{173,-36},{162,-36},{162,-44}}, color={0,0,127}));
  connect(sensor_m_flow.port_b, BypassFeedwaterHeater1.Shell_in) annotation (
      Line(points={{118,8.88178e-16},{118,0},{154,0}}, color={0,127,255}));
  connect(source.ports[1], BypassFeedwaterHeater1.Tube_in)
    annotation (Line(points={{272,-6},{174,-6}}, color={0,127,255}));
  connect(BypassFeedwaterHeater1.Tube_out, sink.ports[1])
    annotation (Line(points={{154,-6},{-12,-6}}, color={0,127,255}));
  connect(LPT2_Bypass1.port_b, Condenser.port_a) annotation (Line(points={{172,
          -52},{242,-52},{242,-68},{241,-68}}, color={0,127,255}));
  connect(BypassFeedwaterHeater1.Shell_out, Condenser.port_a) annotation (Line(
        points={{174,0},{242,0},{242,-68},{241,-68}}, color={0,127,255}));
  connect(LPT2.portLP, Condenser.port_a) annotation (Line(points={{218,40},{242,
          40},{242,-68},{241,-68}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,
            -100},{140,100}}),                                  graphics={
        Rectangle(
          extent={{-2.09756,2},{83.9024,-2}},
          lineColor={0,0,0},
          origin={-45.9024,-64},
          rotation=360,
          fillColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-1.81329,5},{66.1867,-5}},
          lineColor={0,0,0},
          origin={-68.1867,-41},
          rotation=0,
          fillColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-16,3},{16,-3}},
          lineColor={0,0,0},
          fillColor={66,200,200},
          fillPattern=FillPattern.HorizontalCylinder,
          origin={4,30},
          rotation=-90),
        Rectangle(
          extent={{-1.81332,3},{66.1869,-3}},
          lineColor={0,0,0},
          origin={-18.1867,-3},
          rotation=0,
          fillColor={135,135,135},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-70,46},{-22,34}},
          lineColor={0,0,0},
          fillColor={66,200,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Polygon(
          points={{0,16},{0,-14},{30,-32},{30,36},{0,16}},
          lineColor={0,0,0},
          fillColor={0,114,208},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{11,-8},{21,6}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="T"),
        Ellipse(
          extent={{46,12},{74,-14}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-0.4,3},{15.5,-3}},
          lineColor={0,0,0},
          origin={30.4272,-29},
          rotation=0,
          fillColor={0,128,255},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-0.43805,2.7864},{15.9886,-2.7864}},
          lineColor={0,0,0},
          origin={45.2136,-41.989},
          rotation=90,
          fillColor={0,128,255},
          fillPattern=FillPattern.HorizontalCylinder),
        Ellipse(
          extent={{32,-42},{60,-68}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-0.373344,2},{13.6267,-2}},
          lineColor={0,0,0},
          origin={18.3733,-56},
          rotation=0,
          fillColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-0.487802,2},{19.5122,-2}},
          lineColor={0,0,0},
          origin={20,-38.488},
          rotation=-90,
          fillColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-0.243902,2},{9.7562,-2}},
          lineColor={0,0,0},
          origin={-46,-62.244},
          rotation=-90,
          fillColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-0.578156,2.1722},{23.1262,-2.1722}},
          lineColor={0,0,0},
          origin={21.4218,-39.828},
          rotation=180,
          fillColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder),
        Ellipse(
          extent={{-4,-34},{8,-46}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={0,100,199}),
        Polygon(
          points={{-2,-44},{-6,-48},{10,-48},{6,-44},{-2,-44}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.VerticalCylinder),
        Rectangle(
          extent={{-20,46},{6,34}},
          lineColor={0,0,0},
          fillColor={66,200,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Ellipse(
          extent={{-30,49},{-12,31}},
          lineColor={95,95,95},
          fillColor={175,175,175},
          fillPattern=FillPattern.Sphere),
        Rectangle(
          extent={{-20,49},{-22,61}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.VerticalCylinder),
        Rectangle(
          extent={{-30,63},{-12,61}},
          lineColor={0,0,0},
          fillColor={181,0,0},
          fillPattern=FillPattern.HorizontalCylinder),
        Ellipse(
          extent={{-19,49},{-23,31}},
          lineColor={0,0,0},
          fillPattern=FillPattern.VerticalCylinder,
          fillColor={162,162,0}),
        Text(
          extent={{55,-10},{65,4}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="G"),
        Text(
          extent={{41,-62},{51,-48}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="C"),
        Polygon(
          points={{3,-37},{3,-43},{-1,-40},{3,-37}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={255,255,255})}),                            Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},{140,
            100}})),
    experiment(
      StopTime=86400,
      Interval=30,
      __Dymola_Algorithm="Esdirk45a"),
    Documentation(info="<html>
<p>A three-stage turbine rankine cycle with feedwater heating internal to the system</p>
<p>Three bypass ways exist using TBV (Turbine bypass valve), LPTBV1 (Low-Pressure Turbine bypass valve-1), and LPTBV2 (Low-Pressure Turbine bypass valve-2).</p>
</html>"));
end SteamTurbine_L3_HTGR_v2_6_Fail;
