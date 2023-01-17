within NHES.Systems.BalanceOfPlant.Turbine;
package HTGR_RankineCycles
  model SteamTurbine_L2_ClosedFeedHeat_HTGR "Two stage BOP model"
    extends BaseClasses.Partial_SubSystem_C(
      redeclare replaceable
        ControlSystems.CS_SteamTurbine_L2_PressurePowerFeedtemp CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare replaceable Data.Turbine_2 data(InternalBypassValve_p_spring=
            6500000));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      energyDynamics=TRANSFORM.Types.Dynamics.DynamicFreeInitial,
      Q_units_start={1e7},
      eta_mech=data.HPT_efficiency,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=init.HPT_p_a_start,
      p_b_start=init.HPT_p_b_start,
      T_a_start=init.HPT_T_a_start,
      T_b_start=init.HPT_T_b_start,
      m_flow_nominal=data.HPT_nominal_mflow,
      p_inlet_nominal= data.p_in_nominal,
      p_outlet_nominal=data.HPT_p_exit_nominal,
      T_nominal=data.HPT_T_in_nominal)
      annotation (Placement(transformation(extent={{32,22},{52,42}})));

    Fluid.Vessels.IdealCondenser Condenser(
      p= data.p_condensor,
      V_total=data.V_condensor,
      V_liquid_start=init.condensor_V_liquid_start)
      annotation (Placement(transformation(extent={{156,-112},{136,-92}})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));

    TRANSFORM.Fluid.Sensors.Pressure     sensor_p(redeclare package Medium =
          Modelica.Media.Water.StandardWater, redeclare function iconUnit =
          TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar)
                                                         annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-18,60})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      m_flow_start=400,
      dp_nominal=data.valve_TCV_dp_nominal,
      m_flow_nominal=data.valve_TCV_mflow)
                         annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));

    TRANSFORM.Fluid.Machines.SteamTurbine LPT(
      nUnits=1,
      energyDynamics=TRANSFORM.Types.Dynamics.DynamicFreeInitial,
      Q_units_start={1e7},
      eta_mech=data.LPT_efficiency,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=init.LPT_p_a_start,
      p_b_start=init.LPT_p_b_start,
      T_a_start=init.LPT_T_a_start,
      T_b_start=init.LPT_T_b_start,
      partialArc_nominal=2,
      m_flow_nominal=data.LPT_nominal_mflow,
      use_Stodola=true,
      p_inlet_nominal= data.LPT_p_in_nominal,
      p_outlet_nominal=data.LPT_p_exit_nominal,
      T_nominal=data.LPT_T_in_nominal) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={42,-6})));

    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee(redeclare
        package Medium = Modelica.Media.Water.StandardWater, V=data.V_tee,
      p_start=init.tee_p_start,
      T_start=init.moisturesep_T_start)
      annotation (Placement(transformation(extent={{-10,10},{10,-10}},
          rotation=90,
          origin={82,24})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=data.valve_LPT_Bypass_dp_nominal,
      m_flow_nominal=data.valve_LPT_Bypass_mflow)   annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={82,-26})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-134,-40})));

    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             firstfeedpump(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=data.firstfeedpump_p_nominal,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=0,
          origin={108,-144})));

    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      p_start=init.moisturesep_p_start,
      T_start=init.moisturesep_T_start,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
          (V=data.V_moistureseperator))
      annotation (Placement(transformation(extent={{58,30},{78,50}})));

    Fluid.HeatExchangers.Generic_HXs.NTU_HX_SinglePhase MainFeedwaterHeater(
      NTU=data.MainFeedHeater_NTU,
      K_tube=data.MainFeedHeater_K_tube,
      K_shell=data.MainFeedHeater_K_shell,
      redeclare package Tube_medium = Modelica.Media.Water.StandardWater,
      redeclare package Shell_medium = Modelica.Media.Water.StandardWater,
      V_Tube=data.MainFeedHeater_V_tube,
      V_Shell=data.MainFeedHeater_V_shell,
      p_start_tube=init.MainFeedHeater_p_start_tube,
      h_start_tube_inlet=init.MainFeedHeater_h_start_tube_inlet,
      h_start_tube_outlet=init.MainFeedHeater_h_start_tube_outlet,
      p_start_shell=init.MainFeedHeater_p_start_shell,
      h_start_shell_inlet=init.MainFeedHeater_h_start_shell_inlet,
      h_start_shell_outlet=init.MainFeedHeater_h_start_shell_outlet,
      dp_init_tube=init.MainFeedHeater_dp_init_tube,
      dp_init_shell=init.MainFeedHeater_dp_init_shell,
      Q_init=init.MainFeedHeater_Q_init,
      m_start_tube=init.MainFeedHeater_m_start_tube,
      m_start_shell=init.MainFeedHeater_m_start_shell)
      annotation (Placement(transformation(extent={{40,-118},{60,-138}})));

    TRANSFORM.Fluid.Volumes.MixingVolume FeedwaterMixVolume(
      redeclare package Medium = Modelica.Media.Examples.TwoPhaseWater,
      p_start=init.FeedwaterMixVolume_p_start,
      use_T_start=false,
      h_start=init.FeedwaterMixVolume_h_start,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
          (V=data.V_FeedwaterMixVolume),
      nPorts_a=2,
      nPorts_b=1) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={34,-94})));

    Electrical.Generator      generator1(J=data.generator_MoI)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={10,-38})));

    TRANSFORM.Electrical.Sensors.PowerSensor sensorW
      annotation (Placement(transformation(extent={{110,-58},{130,-38}})));

    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance R_feedwater(R=data.R_feedwater,
        redeclare package Medium = Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=180,
          origin={114,-112})));

    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance R_entry(R=data.R_entry,
        redeclare package Medium = Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=180,
          origin={-132,40})));

    TRANSFORM.Fluid.Volumes.MixingVolume header(
      use_T_start=false,
      h_start=init.header_h_start,
      p_start=init.header_p_start,
      nPorts_a=1,
      nPorts_b=1,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
          (V=1),
      redeclare package Medium = Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-122,30},{-102,50}})));

    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=data.p_boundary,
      T=data.T_boundary,
      nPorts=1)
      annotation (Placement(transformation(extent={{-168,64},{-148,84}})));

    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=data.valve_TBV_dp_nominal,
      m_flow_nominal=data.valve_TBV_mflow) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-128,74})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T4(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=180,
          origin={80,-144})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T6(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={20,-132})));

    replaceable Data.Turbine_2_init init(FeedwaterMixVolume_h_start=2e6)
      annotation (Placement(transformation(extent={{68,120},{88,140}})));

    TRANSFORM.Fluid.Machines.Pump                pump_SimpleMassFlow2(
      p_a_start=5500000,
      use_T_start=true,
      h_start=1e6,
      N_nominal=1200,
      dp_nominal=10500000,
      m_flow_nominal=25,
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)                                       annotation (
        Placement(transformation(
          extent={{-11,-11},{11,11}},
          rotation=180,
          origin={-85,-41})));
    TRANSFORM.Fluid.Volumes.MixingVolume FeedwaterMixVolume1(
      redeclare package Medium = Modelica.Media.Examples.TwoPhaseWater,
      p_start=init.FeedwaterMixVolume_p_start,
      use_T_start=false,
      h_start=init.FeedwaterMixVolume_h_start,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
          (V=data.V_FeedwaterMixVolume),
      nPorts_a=1,
      nPorts_b=1) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={86,-112})));
  initial equation

  equation

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{32,38},{30,38},{30,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(LPT.portHP, tee.port_1) annotation (Line(
        points={{48,4},{48,8},{82,8},{82,14}},
        color={0,127,255},
        thickness=0.5));
    connect(tee.port_3, LPT_Bypass.port_a) annotation (Line(
        points={{92,24},{92,0},{82,0},{82,-16}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-30,-60},{-134,-60},{-134,-43.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(HPT.shaft_b, LPT.shaft_a) annotation (Line(
        points={{52,32},{52,14},{42,14},{42,4}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(HPT.portLP, Moisture_Separator.port_a) annotation (Line(
        points={{52,38},{58,38},{58,40},{62,40}},
        color={0,127,255},
        thickness=0.5));
    connect(Moisture_Separator.port_b, tee.port_2) annotation (Line(
        points={{74,40},{82,40},{82,34}},
        color={0,127,255},
        thickness=0.5));

    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensor_p.port, TCV.port_a)
      annotation (Line(points={{-18,50},{-18,40},{-12,40}}, color={0,127,255}));
    connect(LPT_Bypass.port_b, FeedwaterMixVolume.port_a[1])
      annotation (Line(points={{82,-36},{82,-44},{72,-44},{72,-58},{33.5,-58},{
            33.5,-88}},                                     color={0,127,255}));
    connect(Moisture_Separator.port_Liquid, FeedwaterMixVolume.port_a[2])
      annotation (Line(points={{64,36},{64,-44},{72,-44},{72,-58},{34.5,-58},{
            34.5,-88}},
                   color={0,127,255}));

    connect(LPT.shaft_b, generator1.shaft_a)
      annotation (Line(points={{42,-16},{42,-22},{10,-22},{10,-28}},
                                                   color={0,0,0}));
    connect(generator1.portElec, sensorW.port_a) annotation (Line(points={{10,-48},
            {10,-52},{104,-52},{104,-48},{110,-48}}, color={255,0,0}));
    connect(sensorW.port_b, portElec_b) annotation (Line(points={{130,-48},{146,
            -48},{146,0},{160,0}},                     color={255,0,0}));
    connect(FeedwaterMixVolume.port_b[1], MainFeedwaterHeater.Shell_in)
      annotation (Line(points={{34,-100},{34,-126},{40,-126}},
          color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT_Bypass.opening) annotation (
        Line(
        points={{30,100},{114,100},{114,-26},{90,-26}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,60},{-24,60}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(sensorBus.Power, sensorW.W) annotation (Line(
        points={{-30,100},{120,100},{120,-37}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(port_a, R_entry.port_a)
      annotation (Line(points={{-160,40},{-139,40}}, color={0,127,255}));
    connect(R_entry.port_b, header.port_a[1])
      annotation (Line(points={{-125,40},{-118,40}}, color={0,127,255}));
    connect(header.port_b[1], TCV.port_a)
      annotation (Line(points={{-106,40},{-60,40},{-60,40},{-12,40}},
                                                    color={0,127,255}));
    connect(TBV.port_a, TCV.port_a) annotation (Line(points={{-120,74},{-104,74},
            {-104,40},{-12,40}}, color={0,127,255}));
    connect(boundary.ports[1], TBV.port_b)
      annotation (Line(points={{-148,74},{-136,74}}, color={0,127,255}));
    connect(actuatorBus.TBV, TBV.opening) annotation (Line(
        points={{30,100},{-128,100},{-128,80.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(firstfeedpump.port_b, sensor_T4.port_b) annotation (Line(points={{98,-144},
            {90,-144}},                              color={0,127,255}));
    connect(sensor_T4.port_a, MainFeedwaterHeater.Tube_in) annotation (Line(
          points={{70,-144},{64,-144},{64,-132},{60,-132}}, color={0,127,255}));
    connect(MainFeedwaterHeater.Tube_out, sensor_T6.port_a)
      annotation (Line(points={{40,-132},{30,-132}}, color={0,127,255}));
    connect(Condenser.port_b, firstfeedpump.port_a) annotation (Line(points={{146,
            -112},{146,-144},{118,-144}},         color={0,127,255}));
    connect(actuatorBus.Feed_Pump_Speed,pump_SimpleMassFlow2. inputSignal)
      annotation (Line(
        points={{30,100},{-56,100},{-56,-26},{-100,-26},{-100,-56},{-85,-56},{
            -85,-48.7}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(MainFeedwaterHeater.Shell_out, FeedwaterMixVolume1.port_a[1])
      annotation (Line(points={{60,-126},{72,-126},{72,-112},{80,-112}}, color=
            {0,127,255}));
    connect(FeedwaterMixVolume1.port_b[1], R_feedwater.port_a)
      annotation (Line(points={{92,-112},{107,-112}}, color={0,127,255}));
    connect(R_feedwater.port_b, Condenser.port_a) annotation (Line(points={{121,
            -112},{130,-112},{130,-84},{152,-84},{152,-88},{153,-88},{153,-92}},
          color={0,127,255}));
    connect(LPT.portLP, Condenser.port_a) annotation (Line(points={{48,-16},{52,
            -16},{52,-74},{153,-74},{153,-92}}, color={0,127,255}));
    connect(port_b, sensor_T2.port_b)
      annotation (Line(points={{-160,-40},{-144,-40}}, color={0,127,255}));
    connect(sensor_T2.port_a, pump_SimpleMassFlow2.port_b) annotation (Line(
          points={{-124,-40},{-110,-40},{-110,-41},{-96,-41}}, color={0,127,255}));
    connect(pump_SimpleMassFlow2.port_a, sensor_T6.port_b) annotation (Line(
          points={{-74,-41},{-6,-41},{-6,-132},{10,-132}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
            extent={{-24,2},{24,-2}},
            lineColor={0,0,0},
            fillColor={64,164,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={20,-42},
            rotation=180),
          Rectangle(
            extent={{-11.5,3},{11.5,-3}},
            lineColor={0,0,0},
            fillColor={64,164,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={-1,-28.5},
            rotation=90),
          Rectangle(
            extent={{-4.5,2.5},{4.5,-2.5}},
            lineColor={0,0,0},
            fillColor={64,164,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={-8.5,-31.5},
            rotation=360),
          Rectangle(
            extent={{-0.800004,5},{29.1996,-5}},
            lineColor={0,0,0},
            origin={-71.1996,-49},
            rotation=0,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-18,3},{18,-3}},
            lineColor={0,0,0},
            fillColor={66,200,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={-39,28},
            rotation=-90),
          Rectangle(
            extent={{-1.81332,3},{66.1869,-3}},
            lineColor={0,0,0},
            origin={-18.1867,-3},
            rotation=0,
            fillColor={135,135,135},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-70,46},{-36,34}},
            lineColor={0,0,0},
            fillColor={66,200,200},
            fillPattern=FillPattern.HorizontalCylinder),
          Polygon(
            points={{-42,12},{-42,-18},{-12,-36},{-12,32},{-42,12}},
            lineColor={0,0,0},
            fillColor={0,114,208},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-31,-10},{-21,4}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="HPT"),
          Ellipse(
            extent={{46,12},{74,-14}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-0.601938,3},{23.3253,-3}},
            lineColor={0,0,0},
            origin={22.6019,-29},
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
            extent={{-0.341463,2},{13.6587,-2}},
            lineColor={0,0,0},
            origin={20,-44.3415},
            rotation=-90,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-1.41463,2.0001},{56.5851,-2.0001}},
            lineColor={0,0,0},
            origin={18.5851,-46.0001},
            rotation=180,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(
            extent={{-46,-40},{-34,-52}},
            lineColor={0,0,0},
            fillPattern=FillPattern.Sphere,
            fillColor={0,100,199}),
          Polygon(
            points={{-44,-50},{-48,-54},{-32,-54},{-36,-50},{-44,-50}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={0,0,0},
            fillPattern=FillPattern.VerticalCylinder),
          Ellipse(
            extent={{-56,49},{-38,31}},
            lineColor={95,95,95},
            fillColor={175,175,175},
            fillPattern=FillPattern.Sphere),
          Rectangle(
            extent={{-46,49},{-48,61}},
            lineColor={0,0,0},
            fillColor={95,95,95},
            fillPattern=FillPattern.VerticalCylinder),
          Rectangle(
            extent={{-56,63},{-38,61}},
            lineColor={0,0,0},
            fillColor={181,0,0},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(
            extent={{-45,49},{-49,31}},
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
            points={{-39,-43},{-39,-49},{-43,-46},{-39,-43}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={255,255,255}),
          Polygon(
            points={{-4,12},{-4,-18},{26,-36},{26,32},{-4,12}},
            lineColor={0,0,0},
            fillColor={0,114,208},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{7,-10},{17,4}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="LPT"),
          Rectangle(
            extent={{-4,-40},{22,-48}},
            lineColor={238,46,47},
            pattern=LinePattern.None,
            lineThickness=1,
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={28,108,200}),
          Line(
            points={{-4,-44},{22,-44}},
            color={255,0,0},
            thickness=1)}),                                        Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=1000,
        Interval=10,
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>A two stage turbine rankine cycle with feedwater heating internal to the system - can be externally bypassed or LPT can be bypassed both will feedwater heat post bypass</p>
</html>"));
  end SteamTurbine_L2_ClosedFeedHeat_HTGR;

  model HTGR_Rankine_Cycle_Transient
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable ControlSystems.CS_Rankine_Xe100_Based_Secondary CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{78,120},{98,140}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=3000000,
      p_b_start=8000,
      T_a_start=673.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=14000000,
      p_outlet_nominal=2500000,
      T_nominal=673.15)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));
    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{34,-34},{14,-14}})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{56,-58},{36,-38}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-30},{-44,-50}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
    TRANSFORM.Fluid.Sensors.Pressure     sensor_p(redeclare package Medium =
          Modelica.Media.Water.StandardWater, redeclare function iconUnit =
          TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar)
                                                         annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-18,76})));
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
          origin={-62,40})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));
    Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
      annotation (Placement(transformation(extent={{-100,106},{-88,120}})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=3000000,
      p_b_start=8000,
      T_a_start=673.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=2500000,
      p_outlet_nominal=8000,
      T_nominal=673.15) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={46,-6})));
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
          origin={-4,-40})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee(redeclare
        package Medium = Modelica.Media.Water.StandardWater, V=5,
      p_start=2500000,
      T_start=573.15)
      annotation (Placement(transformation(extent={{-10,10},{10,-10}},
          rotation=90,
          origin={76,26})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5)   annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={86,-34})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-70,-40})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-74},{20,-54}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,                         redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{56,30},{76,50}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,-7},{-6,7}},
          rotation=90,
          origin={53,-26})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,-50},{-90,-30}})));
    TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{116,100},{116,-82},{-34,-82},{-34,-47}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-24,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-80,100},{-80,112},{-84,112},{-84,113},{-87.4,113}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(points={{-10,-40},{-24,-40}},
                                      color={0,127,255},
        thickness=0.5));
    connect(LPT.portHP, tee.port_1) annotation (Line(
        points={{52,4},{52,8},{76,8},{76,16}},
        color={0,127,255},
        thickness=0.5));
    connect(tee.port_3, LPT_Bypass.port_a) annotation (Line(
        points={{86,26},{86,-24}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-60,-40},{-44,-40}},color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-118,100},{-118,-62},{-70,-62},{-70,-43.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{46,-58},{46,
            -64},{40,-64}},                                           color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-64},{16,-64},
            {16,-40},{2,-40}},                         color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT.shaft_a) annotation (Line(
        points={{54,34},{54,14},{46,14},{46,4}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(actuatorBus.Divert_Valve_Position, LPT_Bypass.opening) annotation (
        Line(
        points={{30,100},{116,100},{116,-34},{94,-34}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(HPT.portLP, Moisture_Separator.port_a) annotation (Line(
        points={{54,40},{60,40}},
        color={0,127,255},
        thickness=0.5));
    connect(Moisture_Separator.port_b, tee.port_2) annotation (Line(
        points={{72,40},{76,40},{76,36}},
        color={0,127,255},
        thickness=0.5));
    connect(Moisture_Separator.port_Liquid, volume1.port_a) annotation (Line(
        points={{62,36},{62,18},{16,18},{16,-40},{2,-40}},
        color={0,127,255},
        thickness=0.5));
    connect(LPT.portLP, sensor_m_flow1.port_a) annotation (Line(
        points={{52,-16},{52,-20},{53,-20}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{53,-32},{53,-38}},     color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary.ports[1]) annotation (Line(points={{-82,72},{-96,
            72}},                                      color={0,127,255}));
    connect(LPT.shaft_b, generator.shaft) annotation (Line(points={{46,-16},{46,-24.1},
            {34.1,-24.1}}, color={0,0,0}));
    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,40},{-46,40},
            {-46,72},{-66,72}}, color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,40},{-12,40}}, color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,40},{-34,40},
            {-34,62},{-18,62},{-18,66}}, color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-100,40},{-68,40}}, color={0,127,255}));
    connect(sensor_T2.port_b, port_b)
      annotation (Line(points={{-80,-40},{-100,-40}}, color={0,127,255}));
    connect(TBV.opening, actuatorBus.TBV) annotation (Line(points={{-74,78.4},{-74,
            100},{30,100}},       color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT_Bypass.port_b, pump1.port_a)
      annotation (Line(points={{86,-44},{86,-64},{40,-64}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=86400,
        Interval=30,
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end HTGR_Rankine_Cycle_Transient;

  model SteamTurbine_Basic_DirectCoupling_HTGR "Two stage BOP model"
    extends BaseClasses.Partial_SubSystem_C(
      redeclare replaceable
        ControlSystems.CS_SteamTurbine_L2_PressurePowerFeedtemp CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare replaceable Data.TESTurbine data(
        p_condensor=7000,
        V_FeedwaterMixVolume=10,
        V_Header=10,
        R_entry=8e4,
        valve_SHS_mflow=30,
        valve_SHS_dp_nominal=1200000,
        valve_TCV_LPT_mflow=30,
        valve_TCV_LPT_dp_nominal=10000,
        InternalBypassValve_mflow_small=0,
        InternalBypassValve_p_spring=15000000,
        InternalBypassValve_K=40,
        LPT_p_in_nominal=1200000,
        LPT_p_exit_nominal=7000,
        LPT_T_in_nominal=491.15,
        LPT_nominal_mflow=26.83,
        LPT_efficiency=1,
        firstfeedpump_p_nominal=2000000,
        secondfeedpump_p_nominal=2000000));

    replaceable Data.IntermediateTurbineInitialisation init(
      FeedwaterMixVolume_p_start=3000000,
        FeedwaterMixVolume_h_start=2e6,
      InternalBypassValve_dp_start=3500000,
      InternalBypassValve_mflow_start=0.1,
      HPT_p_a_start=3000000,
      HPT_p_b_start=10000,
      HPT_T_a_start=523.15,
      HPT_T_b_start=333.15)
    annotation (Placement(transformation(extent={{68,120},{88,140}})));

    Fluid.Vessels.IdealCondenser Condenser(
      p= data.p_condensor,
      V_total=data.V_condensor,
      V_liquid_start=init.condensor_V_liquid_start)
      annotation (Placement(transformation(extent={{156,-112},{136,-92}})));

    Electrical.Generator      generator1(J=data.generator_MoI)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={84,-92})));

    TRANSFORM.Electrical.Sensors.PowerSensor sensorW
      annotation (Placement(transformation(extent={{110,-58},{130,-38}})));

    Modelica.Mechanics.Rotational.Sensors.PowerSensor powerSensor
      annotation (Placement(transformation(extent={{52,-66},{72,-86}})));

    TRANSFORM.Fluid.Machines.SteamTurbine LPT(
      nUnits=1,
      energyDynamics=TRANSFORM.Types.Dynamics.DynamicFreeInitial,
      Q_units_start={2e7},
      eta_mech=data.LPT_efficiency,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant
          ( eta_nominal=0.9),
      p_a_start=init.LPT_p_a_start,
      p_b_start=init.LPT_p_b_start,
      T_a_start=init.LPT_T_a_start,
      T_b_start=init.LPT_T_b_start,
      m_flow_nominal=data.LPT_nominal_mflow,
      p_inlet_nominal=data.LPT_p_in_nominal,
      p_outlet_nominal=data.LPT_p_exit_nominal,
      T_nominal=data.LPT_T_in_nominal) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={46,-40})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T3(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={134,72})));
    TRANSFORM.Fluid.Valves.ValveLinear TCV_LPT(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      m_flow_start=400,
      dp_nominal=data.valve_TCV_LPT_dp_nominal,
      m_flow_nominal=data.valve_TCV_LPT_mflow) annotation (Placement(
          transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={104,72})));
    TRANSFORM.Fluid.Valves.ValveLinear Discharge_OnOff(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      m_flow_start=400,
      dp_nominal=100000,
      m_flow_nominal=100)
                         annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={126,-146})));
    TRANSFORM.Fluid.Machines.Pump_Controlled firstfeedpump1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_a_start=10000,
      p_b_start=1100000,
      N_nominal=1500,
      dp_nominal=5000000,
      m_flow_nominal=30,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=0,
          origin={94,-142})));
    TRANSFORM.Fluid.Sensors.Pressure     sensor_p(redeclare package Medium =
          Modelica.Media.Water.StandardWater, redeclare function iconUnit =
          TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar)
                                                         annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-124,60})));
  initial equation

  equation

    connect(generator1.portElec, sensorW.port_a) annotation (Line(points={{84,-102},
            {84,-106},{104,-106},{104,-48},{110,-48}},
                                                     color={255,0,0}));
    connect(sensorW.port_b, portElec_b) annotation (Line(points={{130,-48},{146,
            -48},{146,0},{160,0}},                     color={255,0,0}));
    connect(sensorBus.Power, sensorW.W) annotation (Line(
        points={{-30,100},{120,100},{120,-37}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(powerSensor.flange_b, generator1.shaft_a) annotation (Line(points={{72,-76},
            {84,-76},{84,-82}},                      color={0,0,0}));
    connect(LPT.shaft_b, powerSensor.flange_a)
      annotation (Line(points={{46,-50},{46,-76},{52,-76}}, color={0,0,0}));
    connect(LPT.portLP, Condenser.port_a) annotation (Line(points={{52,-50},{52,-58},
            {38,-58},{38,-112},{118,-112},{118,-84},{153,-84},{153,-92}}, color={0,
            127,255}));
    connect(sensorBus.SHS_Return_T, sensor_T3.T) annotation (Line(
        points={{-30,100},{-30,74},{88,74},{88,58},{134,58},{134,68.4}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,-6},{-3,-6}},
        horizontalAlignment=TextAlignment.Right));
    connect(actuatorBus.TCV_SHS, TCV_LPT.opening) annotation (Line(
        points={{30,100},{104,100},{104,78.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(TCV_LPT.port_b, sensor_T3.port_b)
      annotation (Line(points={{112,72},{124,72}}, color={0,127,255}));
    connect(actuatorBus.Discharge_OnOff_Throttle, Discharge_OnOff.opening)
      annotation (Line(
        points={{30,100},{186,100},{186,-132},{126,-132},{126,-139.6}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(firstfeedpump1.port_a, Condenser.port_b) annotation (Line(points={{84,-142},
            {78,-142},{78,-128},{146,-128},{146,-112}},       color={0,127,255}));
    connect(firstfeedpump1.port_b, Discharge_OnOff.port_a) annotation (Line(
          points={{104,-142},{112,-142},{112,-146},{118,-146}}, color={0,127,255}));
    connect(TCV_LPT.port_a, LPT.portHP)
      annotation (Line(points={{96,72},{52,72},{52,-30}}, color={0,127,255}));
    connect(Discharge_OnOff.port_b, port_b) annotation (Line(points={{134,-146},{
            144,-146},{144,-160},{-144,-160},{-144,-40},{-160,-40}}, color={0,127,
            255}));
    connect(actuatorBus.Feed_Pump_Speed, firstfeedpump1.inputSignal) annotation (
        Line(
        points={{30,100},{112,100},{112,102},{206,102},{206,-138},{114,-138},{114,
            -132},{94,-132},{94,-135}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(sensorBus.Steam_Pressure,sensor_p. p) annotation (Line(
        points={{-30,100},{-30,60},{-130,60}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(port_a, sensor_p.port) annotation (Line(points={{-160,40},{-124,40},{
            -124,50}}, color={0,127,255}));
    connect(sensor_p.port, sensor_T3.port_a) annotation (Line(points={{-124,50},{
            -124,40},{150,40},{150,72},{144,72}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
            extent={{-24,2},{24,-2}},
            lineColor={0,0,0},
            fillColor={64,164,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={20,-42},
            rotation=180),
          Rectangle(
            extent={{-11.5,3},{11.5,-3}},
            lineColor={0,0,0},
            fillColor={64,164,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={-1,-28.5},
            rotation=90),
          Rectangle(
            extent={{-4.5,2.5},{4.5,-2.5}},
            lineColor={0,0,0},
            fillColor={64,164,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={-8.5,-31.5},
            rotation=360),
          Rectangle(
            extent={{-0.800004,5},{29.1996,-5}},
            lineColor={0,0,0},
            origin={-71.1996,-49},
            rotation=0,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-18,3},{18,-3}},
            lineColor={0,0,0},
            fillColor={66,200,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={-39,28},
            rotation=-90),
          Rectangle(
            extent={{-1.81332,3},{66.1869,-3}},
            lineColor={0,0,0},
            origin={-18.1867,-3},
            rotation=0,
            fillColor={135,135,135},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-70,46},{-36,34}},
            lineColor={0,0,0},
            fillColor={66,200,200},
            fillPattern=FillPattern.HorizontalCylinder),
          Polygon(
            points={{-42,12},{-42,-18},{-12,-36},{-12,32},{-42,12}},
            lineColor={0,0,0},
            fillColor={0,114,208},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-31,-10},{-21,4}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="HPT"),
          Ellipse(
            extent={{46,12},{74,-14}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-0.601938,3},{23.3253,-3}},
            lineColor={0,0,0},
            origin={22.6019,-29},
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
            extent={{-0.341463,2},{13.6587,-2}},
            lineColor={0,0,0},
            origin={20,-44.3415},
            rotation=-90,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-1.41463,2.0001},{56.5851,-2.0001}},
            lineColor={0,0,0},
            origin={18.5851,-46.0001},
            rotation=180,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(
            extent={{-46,-40},{-34,-52}},
            lineColor={0,0,0},
            fillPattern=FillPattern.Sphere,
            fillColor={0,100,199}),
          Polygon(
            points={{-44,-50},{-48,-54},{-32,-54},{-36,-50},{-44,-50}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={0,0,0},
            fillPattern=FillPattern.VerticalCylinder),
          Ellipse(
            extent={{-56,49},{-38,31}},
            lineColor={95,95,95},
            fillColor={175,175,175},
            fillPattern=FillPattern.Sphere),
          Rectangle(
            extent={{-46,49},{-48,61}},
            lineColor={0,0,0},
            fillColor={95,95,95},
            fillPattern=FillPattern.VerticalCylinder),
          Rectangle(
            extent={{-56,63},{-38,61}},
            lineColor={0,0,0},
            fillColor={181,0,0},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(
            extent={{-45,49},{-49,31}},
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
            points={{-39,-43},{-39,-49},{-43,-46},{-39,-43}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={255,255,255}),
          Polygon(
            points={{-4,12},{-4,-18},{26,-36},{26,32},{-4,12}},
            lineColor={0,0,0},
            fillColor={0,114,208},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{7,-10},{17,4}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="LPT"),
          Rectangle(
            extent={{-4,-40},{22,-48}},
            lineColor={238,46,47},
            pattern=LinePattern.None,
            lineThickness=1,
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={28,108,200}),
          Line(
            points={{-4,-44},{22,-44}},
            color={255,0,0},
            thickness=1)}),                                        Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=1000,
        Interval=10,
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>A two stage turbine rankine cycle with feedwater heating internal to the system - can be externally bypassed or LPT can be bypassed both will feedwater heat post bypass</p>
</html>"));
  end SteamTurbine_Basic_DirectCoupling_HTGR;

  model SteamTurbine_OpenFeedHeat_DivertPowerControl_PowerBoostLoop_HTGR
    "Two stage BOP model"
    extends BaseClasses.Partial_SubSystem_C(
      redeclare replaceable
        ControlSystems.CS_SteamTurbine_L2_PressurePowerFeedtemp CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare replaceable Data.TESTurbine data(
        p_condensor=7000,
        V_FeedwaterMixVolume=25,
        V_Header=10,
        valve_TCV_mflow=67,
        valve_TCV_dp_nominal=100000,
        valve_SHS_mflow=30,
        valve_SHS_dp_nominal=1200000,
        valve_TCV_LPT_mflow=30,
        valve_TCV_LPT_dp_nominal=10000,
        InternalBypassValve_mflow_small=0,
        InternalBypassValve_p_spring=15000000,
        InternalBypassValve_K=40,
        HPT_efficiency=1,
        LPT_efficiency=1,
        firstfeedpump_p_nominal=2000000,
        secondfeedpump_p_nominal=2000000));

    replaceable Data.IntermediateTurbineInitialisation init(
      FeedwaterMixVolume_p_start=3000000,
        FeedwaterMixVolume_h_start=2e6,
      InternalBypassValve_dp_start=3500000,
      InternalBypassValve_mflow_start=0.1,
      HPT_p_a_start=3000000,
      HPT_p_b_start=10000,
      HPT_T_a_start=523.15,
      HPT_T_b_start=333.15)
    annotation (Placement(transformation(extent={{68,120},{88,140}})));

    Fluid.Vessels.IdealCondenser Condenser(
      p= data.p_condensor,
      V_total=data.V_condensor,
      V_liquid_start=init.condensor_V_liquid_start)
      annotation (Placement(transformation(extent={{156,-112},{136,-92}})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)                                    annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));

    TRANSFORM.Fluid.Sensors.Pressure     sensor_p(redeclare package Medium =
          Modelica.Media.Water.StandardWater, redeclare function iconUnit =
          TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar)
                                                         annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-18,60})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      m_flow_start=400,
      dp_nominal=data.valve_TCV_dp_nominal,
      m_flow_nominal=data.valve_TCV_mflow)
                         annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-72,-42})));

    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             firstfeedpump(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=data.firstfeedpump_p_nominal,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=0,
          origin={40,-128})));

    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance R_InternalBypass(R=data.R_bypass,
        redeclare package Medium = Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-24,-2})));

    TRANSFORM.Fluid.Volumes.MixingVolume FeedwaterMixVolume(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=init.FeedwaterMixVolume_p_start,
      use_T_start=true,
      T_start=421.15,
      h_start=init.FeedwaterMixVolume_h_start,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
          (V=data.V_FeedwaterMixVolume),
      nPorts_a=1,
      nPorts_b=3) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-30,-40})));

    Electrical.Generator      generator1(J=data.generator_MoI)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={84,-92})));

    TRANSFORM.Electrical.Sensors.PowerSensor sensorW
      annotation (Placement(transformation(extent={{110,-58},{130,-38}})));

    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance R_entry(R=data.R_entry,
        redeclare package Medium = Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=180,
          origin={-132,40})));

    TRANSFORM.Fluid.Volumes.MixingVolume header(
      use_T_start=false,
      h_start=init.header_h_start,
      p_start=init.header_p_start,
      nPorts_a=1,
      nPorts_b=2,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
          (V=1),
      redeclare package Medium = Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-122,32},{-102,52}})));

    TRANSFORM.Fluid.Machines.Pump                pump_SimpleMassFlow1(
      p_a_start=5500000,
      p_b_start=25000000,
      use_T_start=false,
      T_start=481.15,
      h_start=1e6,
      m_flow_start=50,
      N_nominal=1500,
      dp_nominal=8500000,
      m_flow_nominal=data.controlledfeedpump_mflow_nominal,
      redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_port=true)                                       annotation (
        Placement(transformation(
          extent={{-11,-11},{11,11}},
          rotation=180,
          origin={-109,-41})));

    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=data.p_boundary,
      T=data.T_boundary,
      nPorts=1)
      annotation (Placement(transformation(extent={{-168,64},{-148,84}})));

    TRANSFORM.Fluid.Valves.ValveLinear PRV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=data.valve_TBV_dp_nominal,
      m_flow_nominal=data.valve_TBV_mflow) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-128,74})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T4(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=180,
          origin={0,-128})));

    Modelica.Mechanics.Rotational.Sensors.PowerSensor powerSensor
      annotation (Placement(transformation(extent={{52,-66},{72,-86}})));

    Modelica.Fluid.Interfaces.FluidPort_a port_a1(redeclare package Medium =
          Modelica.Media.Water.StandardWater, m_flow)
      "Fluid connector a (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{-102,-170},{-82,-150}}),
          iconTransformation(extent={{-74,-106},{-54,-86}})));

    TRANSFORM.Fluid.Valves.ValveLinear InternalBypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      m_flow_start=400,
      dp_nominal=1500000,
      m_flow_nominal=15) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-74,22})));
    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      energyDynamics=TRANSFORM.Types.Dynamics.DynamicFreeInitial,
      Q_units_start={2e7},
      eta_mech=data.HPT_efficiency,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant
          ( eta_nominal=0.9),
      p_a_start=init.HPT_p_a_start,
      p_b_start=init.HPT_p_b_start,
      T_a_start=init.HPT_T_a_start,
      T_b_start=init.HPT_T_b_start,
      m_flow_nominal=data.HPT_nominal_mflow,
      p_inlet_nominal=data.p_in_nominal,
      p_outlet_nominal=data.HPT_p_exit_nominal,
      T_nominal=data.HPT_T_in_nominal)
      annotation (Placement(transformation(extent={{32,24},{52,44}})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=data.V_tee,
      p_start=init.tee_p_start,
      T_start=523.15)
      annotation (Placement(transformation(extent={{-10,10},{10,-10}},
          rotation=90,
          origin={90,4})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT(
      nUnits=1,
      energyDynamics=TRANSFORM.Types.Dynamics.DynamicFreeInitial,
      Q_units_start={2e7},
      eta_mech=data.LPT_efficiency,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant
          ( eta_nominal=0.9),
      p_a_start=init.LPT_p_a_start,
      p_b_start=init.LPT_p_b_start,
      T_a_start=init.LPT_T_a_start,
      T_b_start=init.LPT_T_b_start,
      m_flow_nominal=data.LPT_nominal_mflow,
      p_inlet_nominal=data.LPT_p_in_nominal,
      p_outlet_nominal=data.LPT_p_exit_nominal,
      T_nominal=data.LPT_T_in_nominal) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={46,-40})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a2(redeclare package Medium =
          Modelica.Media.Water.StandardWater, m_flow)
      "Fluid connector a (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{150,62},{170,82}}),
          iconTransformation(extent={{88,58},{108,78}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b1(redeclare package Medium =
          Modelica.Media.Water.StandardWater, m_flow)
      "Fluid connector b (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{150,-156},{170,-136}}),
          iconTransformation(extent={{88,-62},{108,-42}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T3(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={134,72})));
    TRANSFORM.Fluid.Valves.ValveLinear TCV_LPT(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      m_flow_start=100,
      dp_nominal=data.valve_TCV_LPT_dp_nominal,
      m_flow_nominal=data.valve_TCV_LPT_mflow) annotation (Placement(
          transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={104,72})));
    TRANSFORM.Fluid.Valves.ValveLinear SHS_charge_control(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      m_flow_start=400,
      dp_nominal=data.valve_SHS_dp_nominal,
      m_flow_nominal=data.valve_SHS_mflow) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-62,-102})));
    TRANSFORM.Fluid.Valves.ValveLinear Discharge_OnOff(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      m_flow_start=400,
      dp_nominal=400000,
      m_flow_nominal=26) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={126,-146})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             firstfeedpump1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=1400000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=0,
          origin={94,-148})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-136,-42})));
  initial equation

  equation

    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-30,12},{-48,12},{-48,-58},{-72,-58},{-72,-45.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));

    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensor_p.port, TCV.port_a)
      annotation (Line(points={{-18,50},{-18,40},{-12,40}}, color={0,127,255}));

    connect(generator1.portElec, sensorW.port_a) annotation (Line(points={{84,-102},
            {84,-106},{104,-106},{104,-48},{110,-48}},
                                                     color={255,0,0}));
    connect(sensorW.port_b, portElec_b) annotation (Line(points={{130,-48},{146,
            -48},{146,0},{160,0}},                     color={255,0,0}));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,60},{-24,60}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(sensorBus.Power, sensorW.W) annotation (Line(
        points={{-30,100},{120,100},{120,-37}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(port_a, R_entry.port_a)
      annotation (Line(points={{-160,40},{-139,40}}, color={0,127,255}));
    connect(R_entry.port_b, header.port_a[1])
      annotation (Line(points={{-125,40},{-122,40},{-122,42},{-118,42}},
                                                     color={0,127,255}));
    connect(header.port_b[1], TCV.port_a)
      annotation (Line(points={{-106,41.5},{-60,41.5},{-60,40},{-12,40}},
                                                    color={0,127,255}));
    connect(PRV.port_a, TCV.port_a) annotation (Line(points={{-120,74},{-104,74},
            {-104,40},{-12,40}}, color={0,127,255}));
    connect(boundary.ports[1],PRV. port_b)
      annotation (Line(points={{-148,74},{-136,74}}, color={0,127,255}));
    connect(actuatorBus.TBV,PRV. opening) annotation (Line(
        points={{30,100},{-128,100},{-128,80.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(firstfeedpump.port_b, sensor_T4.port_b) annotation (Line(points={{30,-128},
            {10,-128}},                              color={0,127,255}));
    connect(pump_SimpleMassFlow1.port_a, sensor_T2.port_b) annotation (Line(
          points={{-98,-41},{-98,-42},{-82,-42}},                      color={0,
            127,255}));
    connect(Condenser.port_b, firstfeedpump.port_a) annotation (Line(points={{146,
            -112},{146,-128},{50,-128}},          color={0,127,255}));
    connect(powerSensor.flange_b, generator1.shaft_a) annotation (Line(points={{72,-76},
            {84,-76},{84,-82}},                      color={0,0,0}));
    connect(sensor_T2.port_a, FeedwaterMixVolume.port_a[1]) annotation (Line(
          points={{-62,-42},{-42,-42},{-42,-40},{-36,-40}}, color={0,127,255}));
    connect(FeedwaterMixVolume.port_b[1], R_InternalBypass.port_b)
      annotation (Line(points={{-24,-40.6667},{-24,-9}},
                                                      color={0,127,255}));
    connect(FeedwaterMixVolume.port_b[2], sensor_T4.port_a) annotation (Line(
          points={{-24,-40},{-20,-40},{-20,-128},{-10,-128}},
          color={0,127,255}));
    connect(InternalBypass.port_a, header.port_b[2]) annotation (Line(points={{
            -82,22},{-94,22},{-94,24},{-106,24},{-106,42.5}}, color={0,127,255}));
    connect(InternalBypass.port_b, R_InternalBypass.port_a) annotation (Line(
          points={{-66,22},{-44,22},{-44,20},{-24,20},{-24,5}}, color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, InternalBypass.opening)
      annotation (Line(
        points={{30,100},{-74,100},{-74,28.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(HPT.shaft_b,LPT. shaft_a) annotation (Line(
        points={{52,34},{56,34},{56,-24},{46,-24},{46,-30}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(LPT.shaft_b, powerSensor.flange_a)
      annotation (Line(points={{46,-50},{46,-76},{52,-76}}, color={0,0,0}));
    connect(LPT.portLP, Condenser.port_a) annotation (Line(points={{52,-50},{52,-58},
            {38,-58},{38,-112},{118,-112},{118,-84},{153,-84},{153,-92}}, color={0,
            127,255}));
    connect(sensor_T1.port_b, HPT.portHP) annotation (Line(points={{28,40},{32,40}},
                            color={0,127,255}));
    connect(HPT.portLP, tee.port_2) annotation (Line(points={{52,40},{72,40},{72,
            36},{90,36},{90,14}}, color={0,127,255}));
    connect(LPT.portHP, tee.port_1) annotation (Line(points={{52,-30},{66,-30},{
            66,-28},{90,-28},{90,-6}}, color={0,127,255}));
    connect(sensorBus.SHS_Return_T, sensor_T3.T) annotation (Line(
        points={{-30,100},{-30,74},{88,74},{88,58},{134,58},{134,68.4}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,-6},{-3,-6}},
        horizontalAlignment=TextAlignment.Right));
    connect(actuatorBus.TCV_SHS, TCV_LPT.opening) annotation (Line(
        points={{30,100},{104,100},{104,78.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(port_a1, SHS_charge_control.port_a) annotation (Line(points={{-92,
            -160},{-92,-102},{-70,-102}}, color={0,127,255}));
    connect(SHS_charge_control.port_b, FeedwaterMixVolume.port_b[3]) annotation (
        Line(points={{-54,-102},{-20,-102},{-20,-39.3333},{-24,-39.3333}}, color=
            {0,127,255}));
    connect(actuatorBus.SHS_throttle, SHS_charge_control.opening) annotation (
        Line(
        points={{30,100},{-90,100},{-90,-84},{-62,-84},{-62,-95.6}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV_LPT.port_b, sensor_T3.port_b)
      annotation (Line(points={{112,72},{124,72}}, color={0,127,255}));
    connect(sensor_T3.port_a, port_a2)
      annotation (Line(points={{144,72},{160,72}}, color={0,127,255}));
    connect(TCV_LPT.port_a, tee.port_3) annotation (Line(points={{96,72},{90,72},
            {90,52},{104,52},{104,4},{100,4}}, color={0,127,255}));
    connect(Discharge_OnOff.port_b, port_b1)
      annotation (Line(points={{134,-146},{160,-146}}, color={0,127,255}));
    connect(actuatorBus.Discharge_OnOff_Throttle, Discharge_OnOff.opening)
      annotation (Line(
        points={{30,100},{186,100},{186,-132},{126,-132},{126,-139.6}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(firstfeedpump1.port_a, Condenser.port_b) annotation (Line(points={{84,
            -148},{78,-148},{78,-128},{146,-128},{146,-112}}, color={0,127,255}));
    connect(firstfeedpump1.port_b, Discharge_OnOff.port_a) annotation (Line(
          points={{104,-148},{112,-148},{112,-146},{118,-146}}, color={0,127,255}));
    connect(actuatorBus.Feed_Pump_Speed, pump_SimpleMassFlow1.inputSignal)
      annotation (Line(
        points={{30,100},{-92,100},{-92,-56},{-109,-56},{-109,-48.7}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(port_b, sensor_m_flow.port_b) annotation (Line(points={{-160,-40},{
            -154,-40},{-154,-42},{-146,-42}}, color={0,127,255}));
    connect(pump_SimpleMassFlow1.port_b, sensor_m_flow.port_a) annotation (Line(
          points={{-120,-41},{-124,-41},{-124,-42},{-126,-42}}, color={0,127,255}));
    connect(sensorBus.Condensor_Output_mflow, sensor_m_flow.m_flow) annotation (
        Line(
        points={{-30,100},{-108,100},{-108,98},{-180,98},{-180,-68},{-136,-68},{
            -136,-45.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
            extent={{-24,2},{24,-2}},
            lineColor={0,0,0},
            fillColor={64,164,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={20,-42},
            rotation=180),
          Rectangle(
            extent={{-11.5,3},{11.5,-3}},
            lineColor={0,0,0},
            fillColor={64,164,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={-1,-28.5},
            rotation=90),
          Rectangle(
            extent={{-4.5,2.5},{4.5,-2.5}},
            lineColor={0,0,0},
            fillColor={64,164,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={-8.5,-31.5},
            rotation=360),
          Rectangle(
            extent={{-0.800004,5},{29.1996,-5}},
            lineColor={0,0,0},
            origin={-71.1996,-49},
            rotation=0,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-18,3},{18,-3}},
            lineColor={0,0,0},
            fillColor={66,200,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={-39,28},
            rotation=-90),
          Rectangle(
            extent={{-1.81332,3},{66.1869,-3}},
            lineColor={0,0,0},
            origin={-18.1867,-3},
            rotation=0,
            fillColor={135,135,135},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-70,46},{-36,34}},
            lineColor={0,0,0},
            fillColor={66,200,200},
            fillPattern=FillPattern.HorizontalCylinder),
          Polygon(
            points={{-42,12},{-42,-18},{-12,-36},{-12,32},{-42,12}},
            lineColor={0,0,0},
            fillColor={0,114,208},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-31,-10},{-21,4}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="HPT"),
          Ellipse(
            extent={{46,12},{74,-14}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-0.601938,3},{23.3253,-3}},
            lineColor={0,0,0},
            origin={22.6019,-29},
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
            extent={{-0.341463,2},{13.6587,-2}},
            lineColor={0,0,0},
            origin={20,-44.3415},
            rotation=-90,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-1.41463,2.0001},{56.5851,-2.0001}},
            lineColor={0,0,0},
            origin={18.5851,-46.0001},
            rotation=180,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(
            extent={{-46,-40},{-34,-52}},
            lineColor={0,0,0},
            fillPattern=FillPattern.Sphere,
            fillColor={0,100,199}),
          Polygon(
            points={{-44,-50},{-48,-54},{-32,-54},{-36,-50},{-44,-50}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={0,0,0},
            fillPattern=FillPattern.VerticalCylinder),
          Ellipse(
            extent={{-56,49},{-38,31}},
            lineColor={95,95,95},
            fillColor={175,175,175},
            fillPattern=FillPattern.Sphere),
          Rectangle(
            extent={{-46,49},{-48,61}},
            lineColor={0,0,0},
            fillColor={95,95,95},
            fillPattern=FillPattern.VerticalCylinder),
          Rectangle(
            extent={{-56,63},{-38,61}},
            lineColor={0,0,0},
            fillColor={181,0,0},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(
            extent={{-45,49},{-49,31}},
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
            points={{-39,-43},{-39,-49},{-43,-46},{-39,-43}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={255,255,255}),
          Polygon(
            points={{-4,12},{-4,-18},{26,-36},{26,32},{-4,12}},
            lineColor={0,0,0},
            fillColor={0,114,208},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{7,-10},{17,4}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="LPT"),
          Rectangle(
            extent={{-4,-40},{22,-48}},
            lineColor={238,46,47},
            pattern=LinePattern.None,
            lineThickness=1,
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={28,108,200}),
          Line(
            points={{-4,-44},{22,-44}},
            color={255,0,0},
            thickness=1)}),                                        Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=1000,
        Interval=10,
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>A two stage turbine rankine cycle with feedwater heating internal to the system - can be externally bypassed or LPT can be bypassed both will feedwater heat post bypass</p>
</html>"));
  end SteamTurbine_OpenFeedHeat_DivertPowerControl_PowerBoostLoop_HTGR;

  model SteamTurbine_OpenFeedHeat_DivertPowerControl_HTGR "Two stage BOP model"
    extends BaseClasses.Partial_SubSystem_C(
      redeclare replaceable
        ControlSystems.CS_SteamTurbine_L2_PressurePowerFeedtemp CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare replaceable Data.TESTurbine data(
        p_condensor=8000,
        V_FeedwaterMixVolume=25,
        V_Header=10,
        valve_TCV_mflow=67,
        valve_TCV_dp_nominal=100000,
        valve_SHS_mflow=30,
        valve_SHS_dp_nominal=600000,
        valve_TCV_LPT_mflow=30,
        valve_TCV_LPT_dp_nominal=10000,
        InternalBypassValve_mflow_small=0,
        InternalBypassValve_p_spring=15000000,
        InternalBypassValve_K=40,
        HPT_p_exit_nominal=700000,
        HPT_T_in_nominal=579.15,
        HPT_nominal_mflow=67,
        HPT_efficiency=1,
        LPT_p_in_nominal=700000,
        LPT_p_exit_nominal=7000,
        LPT_T_in_nominal=523.15,
        LPT_nominal_mflow=20,
        LPT_efficiency=1,
        firstfeedpump_p_nominal=2500000,
        secondfeedpump_p_nominal=2000000));

    replaceable Data.IntermediateTurbineInitialisation init(
      FeedwaterMixVolume_p_start=3000000,
        FeedwaterMixVolume_h_start=2e6,
      InternalBypassValve_dp_start=3500000,
      InternalBypassValve_mflow_start=0.1,
      HPT_p_a_start=3000000,
      HPT_p_b_start=10000,
      HPT_T_a_start=523.15,
      HPT_T_b_start=333.15)
    annotation (Placement(transformation(extent={{68,120},{88,140}})));

    Fluid.Vessels.IdealCondenser Condenser(
      p= data.p_condensor,
      V_total=data.V_condensor,
      V_liquid_start=init.condensor_V_liquid_start)
      annotation (Placement(transformation(extent={{156,-112},{136,-92}})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)                                    annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));

    TRANSFORM.Fluid.Sensors.Pressure     sensor_p(redeclare package Medium =
          Modelica.Media.Water.StandardWater, redeclare function iconUnit =
          TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar)
                                                         annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-18,60})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      m_flow_start=400,
      dp_nominal=data.valve_TCV_dp_nominal,
      m_flow_nominal=data.valve_TCV_mflow)
                         annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-72,-42})));

    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             firstfeedpump(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=data.firstfeedpump_p_nominal,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=0,
          origin={40,-128})));

    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance R_InternalBypass(R=data.R_bypass,
        redeclare package Medium = Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-24,-2})));

    TRANSFORM.Fluid.Volumes.MixingVolume FeedwaterMixVolume(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=init.FeedwaterMixVolume_p_start,
      use_T_start=true,
      T_start=421.15,
      h_start=init.FeedwaterMixVolume_h_start,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
          (V=data.V_FeedwaterMixVolume),
      nPorts_a=1,
      nPorts_b=3) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-30,-40})));

    Electrical.Generator      generator1(J=data.generator_MoI)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={84,-92})));

    TRANSFORM.Electrical.Sensors.PowerSensor sensorW
      annotation (Placement(transformation(extent={{110,-58},{130,-38}})));

    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance R_entry(R=data.R_entry,
        redeclare package Medium = Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=180,
          origin={-132,40})));

    TRANSFORM.Fluid.Volumes.MixingVolume header(
      use_T_start=false,
      h_start=init.header_h_start,
      p_start=init.header_p_start,
      nPorts_a=1,
      nPorts_b=2,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
          (V=1),
      redeclare package Medium = Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-122,32},{-102,52}})));

    TRANSFORM.Fluid.Machines.Pump                pump_SimpleMassFlow1(
      N_nominal=1500,
      dp_nominal=CS.data.p_steam,
      m_flow_nominal=data.controlledfeedpump_mflow_nominal,
      redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_port=true)                                       annotation (
        Placement(transformation(
          extent={{-11,-11},{11,11}},
          rotation=180,
          origin={-109,-41})));

    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=data.p_boundary,
      T=data.T_boundary,
      nPorts=1)
      annotation (Placement(transformation(extent={{-168,64},{-148,84}})));

    TRANSFORM.Fluid.Valves.ValveLinear PRV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=data.valve_TBV_dp_nominal,
      m_flow_nominal=data.valve_TBV_mflow) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-128,74})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T4(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=180,
          origin={0,-128})));

    Modelica.Mechanics.Rotational.Sensors.PowerSensor powerSensor
      annotation (Placement(transformation(extent={{52,-66},{72,-86}})));

    Modelica.Fluid.Interfaces.FluidPort_a port_a1(redeclare package Medium =
          Modelica.Media.Water.StandardWater, m_flow)
      "Fluid connector a (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{-102,-170},{-82,-150}}),
          iconTransformation(extent={{-74,-106},{-54,-86}})));

    TRANSFORM.Fluid.Valves.ValveLinear InternalBypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      m_flow_start=400,
      dp_nominal=1000000,
      m_flow_nominal=15) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-74,22})));
    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      energyDynamics=TRANSFORM.Types.Dynamics.DynamicFreeInitial,
      eta_mech=data.HPT_efficiency,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=init.HPT_p_a_start,
      p_b_start=init.HPT_p_b_start,
      T_a_start=init.HPT_T_a_start,
      T_b_start=init.HPT_T_b_start,
      m_flow_nominal=data.HPT_nominal_mflow,
      p_inlet_nominal=data.p_in_nominal,
      p_outlet_nominal=data.HPT_p_exit_nominal,
      T_nominal=data.HPT_T_in_nominal)
      annotation (Placement(transformation(extent={{32,24},{52,44}})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=data.V_tee,
      p_start=init.tee_p_start)
      annotation (Placement(transformation(extent={{-10,10},{10,-10}},
          rotation=90,
          origin={90,4})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT(
      nUnits=1,
      energyDynamics=TRANSFORM.Types.Dynamics.DynamicFreeInitial,
      eta_mech=data.LPT_efficiency,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant
          ( eta_nominal=0.9),
      p_a_start=init.LPT_p_a_start,
      p_b_start=init.LPT_p_b_start,
      T_a_start=init.LPT_T_a_start,
      T_b_start=init.LPT_T_b_start,
      m_flow_nominal=data.LPT_nominal_mflow,
      p_inlet_nominal=data.LPT_p_in_nominal,
      p_outlet_nominal=data.LPT_p_exit_nominal,
      T_nominal=data.LPT_T_in_nominal) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={46,-40})));
    TRANSFORM.Fluid.Valves.ValveLinear SHS_charge_control(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      m_flow_start=400,
      dp_nominal=data.valve_SHS_dp_nominal,
      m_flow_nominal=data.valve_SHS_mflow) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-62,-102})));
    TRANSFORM.Fluid.Valves.ValveLinear Discharge_OnOff(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      m_flow_start=400,
      dp_nominal=100000000,
      m_flow_nominal=20) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={126,-146})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-136,-42})));
    Modelica.Blocks.Sources.Constant const(k=1)
      annotation (Placement(transformation(extent={{50,-156},{70,-136}})));
    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance R_entry1(R=10,
        redeclare package Medium = Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=270,
          origin={-92,-128})));
  initial equation

  equation

    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-30,12},{-48,12},{-48,-58},{-72,-58},{-72,-45.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));

    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensor_p.port, TCV.port_a)
      annotation (Line(points={{-18,50},{-18,40},{-12,40}}, color={0,127,255}));

    connect(generator1.portElec, sensorW.port_a) annotation (Line(points={{84,-102},
            {84,-106},{104,-106},{104,-48},{110,-48}},
                                                     color={255,0,0}));
    connect(sensorW.port_b, portElec_b) annotation (Line(points={{130,-48},{146,
            -48},{146,0},{160,0}},                     color={255,0,0}));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,60},{-24,60}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(sensorBus.Power, sensorW.W) annotation (Line(
        points={{-30,100},{120,100},{120,-37}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(port_a, R_entry.port_a)
      annotation (Line(points={{-160,40},{-139,40}}, color={0,127,255}));
    connect(R_entry.port_b, header.port_a[1])
      annotation (Line(points={{-125,40},{-122,40},{-122,42},{-118,42}},
                                                     color={0,127,255}));
    connect(header.port_b[1], TCV.port_a)
      annotation (Line(points={{-106,41.5},{-60,41.5},{-60,40},{-12,40}},
                                                    color={0,127,255}));
    connect(PRV.port_a, TCV.port_a) annotation (Line(points={{-120,74},{-104,74},
            {-104,40},{-12,40}}, color={0,127,255}));
    connect(boundary.ports[1],PRV. port_b)
      annotation (Line(points={{-148,74},{-136,74}}, color={0,127,255}));
    connect(actuatorBus.TBV,PRV. opening) annotation (Line(
        points={{30,100},{-128,100},{-128,80.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(firstfeedpump.port_b, sensor_T4.port_b) annotation (Line(points={{30,-128},
            {10,-128}},                              color={0,127,255}));
    connect(pump_SimpleMassFlow1.port_a, sensor_T2.port_b) annotation (Line(
          points={{-98,-41},{-98,-42},{-82,-42}},                      color={0,
            127,255}));
    connect(Condenser.port_b, firstfeedpump.port_a) annotation (Line(points={{146,
            -112},{146,-128},{50,-128}},          color={0,127,255}));
    connect(powerSensor.flange_b, generator1.shaft_a) annotation (Line(points={{72,-76},
            {84,-76},{84,-82}},                      color={0,0,0}));
    connect(sensor_T2.port_a, FeedwaterMixVolume.port_a[1]) annotation (Line(
          points={{-62,-42},{-42,-42},{-42,-40},{-36,-40}}, color={0,127,255}));
    connect(FeedwaterMixVolume.port_b[1], R_InternalBypass.port_b)
      annotation (Line(points={{-24,-40.6667},{-24,-9}},
                                                      color={0,127,255}));
    connect(FeedwaterMixVolume.port_b[2], sensor_T4.port_a) annotation (Line(
          points={{-24,-40},{-20,-40},{-20,-128},{-10,-128}},
          color={0,127,255}));
    connect(InternalBypass.port_a, header.port_b[2]) annotation (Line(points={{
            -82,22},{-94,22},{-94,24},{-106,24},{-106,42.5}}, color={0,127,255}));
    connect(InternalBypass.port_b, R_InternalBypass.port_a) annotation (Line(
          points={{-66,22},{-44,22},{-44,20},{-24,20},{-24,5}}, color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, InternalBypass.opening)
      annotation (Line(
        points={{30,100},{-74,100},{-74,28.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(HPT.shaft_b,LPT. shaft_a) annotation (Line(
        points={{52,34},{56,34},{56,-24},{46,-24},{46,-30}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(LPT.shaft_b, powerSensor.flange_a)
      annotation (Line(points={{46,-50},{46,-76},{52,-76}}, color={0,0,0}));
    connect(LPT.portLP, Condenser.port_a) annotation (Line(points={{52,-50},{52,-58},
            {38,-58},{38,-112},{118,-112},{118,-84},{153,-84},{153,-92}}, color={0,
            127,255}));
    connect(sensor_T1.port_b, HPT.portHP) annotation (Line(points={{28,40},{32,40}},
                            color={0,127,255}));
    connect(HPT.portLP, tee.port_2) annotation (Line(points={{52,40},{72,40},{72,
            36},{90,36},{90,14}}, color={0,127,255}));
    connect(LPT.portHP, tee.port_1) annotation (Line(points={{52,-30},{66,-30},{
            66,-28},{90,-28},{90,-6}}, color={0,127,255}));
    connect(SHS_charge_control.port_b, FeedwaterMixVolume.port_b[3]) annotation (
        Line(points={{-54,-102},{-20,-102},{-20,-39.3333},{-24,-39.3333}}, color=
            {0,127,255}));
    connect(actuatorBus.SHS_throttle, SHS_charge_control.opening) annotation (
        Line(
        points={{30,100},{-90,100},{-90,-84},{-62,-84},{-62,-95.6}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump_SimpleMassFlow1.inputSignal)
      annotation (Line(
        points={{30,100},{-92,100},{-92,-56},{-109,-56},{-109,-48.7}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(port_b, sensor_m_flow.port_b) annotation (Line(points={{-160,-40},{
            -154,-40},{-154,-42},{-146,-42}}, color={0,127,255}));
    connect(pump_SimpleMassFlow1.port_b, sensor_m_flow.port_a) annotation (Line(
          points={{-120,-41},{-124,-41},{-124,-42},{-126,-42}}, color={0,127,255}));
    connect(sensorBus.Condensor_Output_mflow, sensor_m_flow.m_flow) annotation (
        Line(
        points={{-30,100},{-108,100},{-108,98},{-180,98},{-180,-68},{-136,-68},{
            -136,-45.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(const.y, Discharge_OnOff.opening) annotation (Line(points={{71,-146},
            {76,-146},{76,-132},{126,-132},{126,-139.6}}, color={0,0,127}));
    connect(Discharge_OnOff.port_a, firstfeedpump.port_a) annotation (Line(points=
           {{118,-146},{114,-146},{114,-128},{50,-128}}, color={0,127,255}));
    connect(Discharge_OnOff.port_b, tee.port_3)
      annotation (Line(points={{134,-146},{134,4},{100,4}}, color={0,127,255}));
    connect(port_a1, R_entry1.port_a)
      annotation (Line(points={{-92,-160},{-92,-135}}, color={0,127,255}));
    connect(R_entry1.port_b, SHS_charge_control.port_a) annotation (Line(points={
            {-92,-121},{-92,-102},{-70,-102}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
            extent={{-24,2},{24,-2}},
            lineColor={0,0,0},
            fillColor={64,164,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={20,-42},
            rotation=180),
          Rectangle(
            extent={{-11.5,3},{11.5,-3}},
            lineColor={0,0,0},
            fillColor={64,164,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={-1,-28.5},
            rotation=90),
          Rectangle(
            extent={{-4.5,2.5},{4.5,-2.5}},
            lineColor={0,0,0},
            fillColor={64,164,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={-8.5,-31.5},
            rotation=360),
          Rectangle(
            extent={{-0.800004,5},{29.1996,-5}},
            lineColor={0,0,0},
            origin={-71.1996,-49},
            rotation=0,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-18,3},{18,-3}},
            lineColor={0,0,0},
            fillColor={66,200,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={-39,28},
            rotation=-90),
          Rectangle(
            extent={{-1.81332,3},{66.1869,-3}},
            lineColor={0,0,0},
            origin={-18.1867,-3},
            rotation=0,
            fillColor={135,135,135},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-70,46},{-36,34}},
            lineColor={0,0,0},
            fillColor={66,200,200},
            fillPattern=FillPattern.HorizontalCylinder),
          Polygon(
            points={{-42,12},{-42,-18},{-12,-36},{-12,32},{-42,12}},
            lineColor={0,0,0},
            fillColor={0,114,208},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-31,-10},{-21,4}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="HPT"),
          Ellipse(
            extent={{46,12},{74,-14}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-0.601938,3},{23.3253,-3}},
            lineColor={0,0,0},
            origin={22.6019,-29},
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
            extent={{-0.341463,2},{13.6587,-2}},
            lineColor={0,0,0},
            origin={20,-44.3415},
            rotation=-90,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-1.41463,2.0001},{56.5851,-2.0001}},
            lineColor={0,0,0},
            origin={18.5851,-46.0001},
            rotation=180,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(
            extent={{-46,-40},{-34,-52}},
            lineColor={0,0,0},
            fillPattern=FillPattern.Sphere,
            fillColor={0,100,199}),
          Polygon(
            points={{-44,-50},{-48,-54},{-32,-54},{-36,-50},{-44,-50}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={0,0,0},
            fillPattern=FillPattern.VerticalCylinder),
          Ellipse(
            extent={{-56,49},{-38,31}},
            lineColor={95,95,95},
            fillColor={175,175,175},
            fillPattern=FillPattern.Sphere),
          Rectangle(
            extent={{-46,49},{-48,61}},
            lineColor={0,0,0},
            fillColor={95,95,95},
            fillPattern=FillPattern.VerticalCylinder),
          Rectangle(
            extent={{-56,63},{-38,61}},
            lineColor={0,0,0},
            fillColor={181,0,0},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(
            extent={{-45,49},{-49,31}},
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
            points={{-39,-43},{-39,-49},{-43,-46},{-39,-43}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={255,255,255}),
          Polygon(
            points={{-4,12},{-4,-18},{26,-36},{26,32},{-4,12}},
            lineColor={0,0,0},
            fillColor={0,114,208},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{7,-10},{17,4}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="LPT"),
          Rectangle(
            extent={{-4,-40},{22,-48}},
            lineColor={238,46,47},
            pattern=LinePattern.None,
            lineThickness=1,
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={28,108,200}),
          Line(
            points={{-4,-44},{22,-44}},
            color={255,0,0},
            thickness=1)}),                                        Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=1000,
        Interval=10,
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>A two stage turbine rankine cycle with feedwater heating internal to the system - can be externally bypassed or LPT can be bypassed both will feedwater heat post bypass</p>
</html>"));
  end SteamTurbine_OpenFeedHeat_DivertPowerControl_HTGR;

  model HTGR_Rankine_Cycle
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable ControlSystems.CS_Rankine_Xe100_Based_Secondary CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{78,120},{98,140}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=3000000,
      p_b_start=8000,
      T_a_start=673.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=14000000,
      p_outlet_nominal=2500000,
      T_nominal=673.15)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));
    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{34,-34},{14,-14}})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=75,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{56,-58},{36,-38}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-30},{-44,-50}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
    TRANSFORM.Fluid.Sensors.Pressure     sensor_p(redeclare package Medium =
          Modelica.Media.Water.StandardWater, redeclare function iconUnit =
          TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar)
                                                         annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-18,76})));
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
          origin={-62,40})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));
    Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
      annotation (Placement(transformation(extent={{-100,106},{-88,120}})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=3000000,
      p_b_start=8000,
      T_a_start=673.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=2500000,
      p_outlet_nominal=8000,
      T_nominal=673.15) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={46,-6})));
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
          origin={-4,-40})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee(redeclare
        package Medium = Modelica.Media.Water.StandardWater, V=5)
      annotation (Placement(transformation(extent={{-10,10},{10,-10}},
          rotation=90,
          origin={76,26})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=2.5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={86,-34})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-70,-40})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=2000000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-74},{20,-54}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator(redeclare package Medium =
          Modelica.Media.Water.StandardWater, redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{56,30},{76,50}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,-7},{-6,7}},
          rotation=90,
          origin={53,-26})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,-50},{-90,-30}})));
    TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{116,100},{116,-82},{-34,-82},{-34,-47}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-24,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-80,100},{-80,112},{-84,112},{-84,113},{-87.4,113}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(points={{-10,-40},{-24,-40}},
                                      color={0,127,255},
        thickness=0.5));
    connect(LPT.portHP, tee.port_1) annotation (Line(
        points={{52,4},{52,8},{76,8},{76,16}},
        color={0,127,255},
        thickness=0.5));
    connect(tee.port_3, LPT_Bypass.port_a) annotation (Line(
        points={{86,26},{86,-24}},
        color={0,127,255},
        thickness=0.5));
    connect(LPT_Bypass.port_b, volume1.port_a) annotation (Line(
        points={{86,-44},{86,-72},{16,-72},{16,-40},{2,-40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-60,-40},{-44,-40}},color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-118,100},{-118,-62},{-70,-62},{-70,-43.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{46,-58},{46,
            -64},{40,-64}},                                           color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-64},{16,-64},
            {16,-40},{2,-40}},                         color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT.shaft_a) annotation (Line(
        points={{54,34},{54,14},{46,14},{46,4}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(actuatorBus.Divert_Valve_Position, LPT_Bypass.opening) annotation (
        Line(
        points={{30,100},{116,100},{116,-34},{94,-34}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(HPT.portLP, Moisture_Separator.port_a) annotation (Line(
        points={{54,40},{60,40}},
        color={0,127,255},
        thickness=0.5));
    connect(Moisture_Separator.port_b, tee.port_2) annotation (Line(
        points={{72,40},{76,40},{76,36}},
        color={0,127,255},
        thickness=0.5));
    connect(Moisture_Separator.port_Liquid, volume1.port_a) annotation (Line(
        points={{62,36},{62,18},{16,18},{16,-40},{2,-40}},
        color={0,127,255},
        thickness=0.5));
    connect(LPT.portLP, sensor_m_flow1.port_a) annotation (Line(
        points={{52,-16},{52,-20},{53,-20}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{53,-32},{53,-38}},     color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary.ports[1]) annotation (Line(points={{-82,72},{-96,
            72}},                                      color={0,127,255}));
    connect(LPT.shaft_b, generator.shaft) annotation (Line(points={{46,-16},{46,-24.1},
            {34.1,-24.1}}, color={0,0,0}));
    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,40},{-46,40},
            {-46,72},{-66,72}}, color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,40},{-12,40}}, color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,40},{-34,40},
            {-34,62},{-18,62},{-18,66}}, color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-100,40},{-68,40}}, color={0,127,255}));
    connect(sensor_T2.port_b, port_b)
      annotation (Line(points={{-80,-40},{-100,-40}}, color={0,127,255}));
    connect(TBV.opening, actuatorBus.TBV) annotation (Line(points={{-74,78.4},{-74,
            100},{30,100}},       color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=86400,
        Interval=30,
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end HTGR_Rankine_Cycle;

  model HTGR_Rankine_Cycle_Transient_JY_v1_step1_comp
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable ControlSystems.CS_Rankine_Xe100_Based_Secondary CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{78,120},{98,140}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_2stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_2stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_2stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_2stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_2stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_2stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_2stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{34,-34},{14,-14}})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{56,-58},{36,-38}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-30},{-44,-50}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
    TRANSFORM.Fluid.Sensors.Pressure     sensor_p(redeclare package Medium =
          Modelica.Media.Water.StandardWater, redeclare function iconUnit =
          TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar)
                                                         annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-18,76})));
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
          origin={-62,40})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));
    Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
      annotation (Placement(transformation(extent={{-100,106},{-88,120}})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_2stage.LPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_2stage.LPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_2stage.LPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_2stage.LPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_2stage.LPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_2stage.LPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_2stage.LPT_T_inlet)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={46,-6})));

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
          origin={-4,-40})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee(redeclare
        package Medium = Modelica.Media.Water.StandardWater, V=5,
      p_start=1500000,
      T_start=573.15)
      annotation (Placement(transformation(extent={{-10,10},{10,-10}},
          rotation=90,
          origin={76,26})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5)   annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={86,-34})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-70,-40})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-74},{20,-54}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      p_start=dataInitial_HTGR_BoP_2stage.HPT_P_outlet,
      T_start=dataInitial_HTGR_BoP_2stage.HPT_T_outlet,
                                              redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{56,30},{76,50}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,-7},{-6,7}},
          rotation=90,
          origin={53,-26})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,-50},{-90,-30}})));
    TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_BoP_2stage
      dataInitial_HTGR_BoP_2stage
      annotation (Placement(transformation(extent={{110,120},{130,140}})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{116,100},{116,-82},{-34,-82},{-34,-47}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-24,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-80,100},{-80,112},{-84,112},{-84,113},{-87.4,113}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(points={{-10,-40},{-24,-40}},
                                      color={0,127,255},
        thickness=0.5));
    connect(LPT.portHP, tee.port_1) annotation (Line(
        points={{52,4},{52,8},{76,8},{76,16}},
        color={0,127,255},
        thickness=0.5));
    connect(tee.port_3, LPT_Bypass.port_a) annotation (Line(
        points={{86,26},{86,-24}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-60,-40},{-44,-40}},color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-118,100},{-118,-62},{-70,-62},{-70,-43.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{46,-58},{46,
            -64},{40,-64}},                                           color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-64},{16,-64},
            {16,-40},{2,-40}},                         color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT.shaft_a) annotation (Line(
        points={{54,34},{54,14},{46,14},{46,4}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(actuatorBus.Divert_Valve_Position, LPT_Bypass.opening) annotation (
        Line(
        points={{30,100},{116,100},{116,-34},{94,-34}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(HPT.portLP, Moisture_Separator.port_a) annotation (Line(
        points={{54,40},{60,40}},
        color={0,127,255},
        thickness=0.5));
    connect(Moisture_Separator.port_b, tee.port_2) annotation (Line(
        points={{72,40},{76,40},{76,36}},
        color={0,127,255},
        thickness=0.5));
    connect(Moisture_Separator.port_Liquid, volume1.port_a) annotation (Line(
        points={{62,36},{62,18},{16,18},{16,-40},{2,-40}},
        color={0,127,255},
        thickness=0.5));
    connect(LPT.portLP, sensor_m_flow1.port_a) annotation (Line(
        points={{52,-16},{52,-20},{53,-20}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{53,-32},{53,-38}},     color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary.ports[1]) annotation (Line(points={{-82,72},{-96,
            72}},                                      color={0,127,255}));
    connect(LPT.shaft_b, generator.shaft) annotation (Line(points={{46,-16},{46,-24.1},
            {34.1,-24.1}}, color={0,0,0}));
    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,40},{-46,40},
            {-46,72},{-66,72}}, color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,40},{-12,40}}, color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,40},{-34,40},
            {-34,62},{-18,62},{-18,66}}, color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-100,40},{-68,40}}, color={0,127,255}));
    connect(sensor_T2.port_b, port_b)
      annotation (Line(points={{-80,-40},{-100,-40}}, color={0,127,255}));
    connect(TBV.opening, actuatorBus.TBV) annotation (Line(points={{-74,78.4},{-74,
            100},{30,100}},       color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT_Bypass.port_b, pump1.port_a)
      annotation (Line(points={{86,-44},{86,-64},{40,-64}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=86400,
        Interval=30,
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end HTGR_Rankine_Cycle_Transient_JY_v1_step1_comp;

  model HTGR_Rankine_Cycle_Transient_JY_v1_step3_comp
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable ControlSystems.CS_Rankine_Xe100_Based_Secondary CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{78,120},{98,140}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{256,24},{276,44}})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{246,-62},{226,-42}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-30},{-44,-50}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
    TRANSFORM.Fluid.Sensors.Pressure     sensor_p(redeclare package Medium =
          Modelica.Media.Water.StandardWater, redeclare function iconUnit =
          TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar)
                                                         annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-18,76})));
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
          origin={-62,40})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));
    Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
      annotation (Placement(transformation(extent={{-100,106},{-88,120}})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT1(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
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
          origin={-4,-40})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-34})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-70,-40})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-74},{20,-54}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{60,40},{80,60}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-23})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,-50},{-90,-30}})));
    TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
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
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,-4})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{116,120},{136,140}})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{300,100},{300,-78},{-34,-78},{-34,-47}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-24,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-80,100},{-80,112},{-84,112},{-84,113},{-87.4,113}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(points={{-10,-40},{-24,-40}},
                                      color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-24}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-60,-40},{-44,-40}},color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-118,100},{-118,-62},{-70,-62},{-70,-43.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{236,-62},
            {236,-64},{40,-64}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-64},{16,-64},
            {16,-40},{2,-40}},                         color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(actuatorBus.Divert_Valve_Position, LPT1_Bypass.opening) annotation (
        Line(
        points={{30,100},{288,100},{288,-34},{102,-34}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(HPT.portLP, Moisture_Separator1.port_a) annotation (Line(
        points={{54,40},{58,40},{58,50},{64,50}},
        color={0,127,255},
        thickness=0.5));
    connect(Moisture_Separator1.port_b, tee1.port_2) annotation (Line(
        points={{76,50},{84,50}},
        color={0,127,255},
        thickness=0.5));
    connect(Moisture_Separator1.port_Liquid, volume1.port_a) annotation (Line(
        points={{66,46},{66,18},{16,18},{16,-40},{2,-40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-30},{242,-42},{243,-42}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary.ports[1]) annotation (Line(points={{-82,72},{-96,
            72}},                                      color={0,127,255}));
    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,40},{-46,40},
            {-46,72},{-66,72}}, color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,40},{-12,40}}, color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,40},{-34,40},
            {-34,62},{-18,62},{-18,66}}, color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-100,40},{-68,40}}, color={0,127,255}));
    connect(sensor_T2.port_b, port_b)
      annotation (Line(points={{-80,-40},{-100,-40}}, color={0,127,255}));
    connect(TBV.opening, actuatorBus.TBV) annotation (Line(points={{-74,78.4},{-74,
            100},{30,100}},       color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1_Bypass.port_b, pump1.port_a)
      annotation (Line(points={{94,-44},{94,-64},{40,-64}}, color={0,127,255}));
    connect(LPT1.shaft_b, LPT2.shaft_a)
      annotation (Line(points={{138,34},{198,34}}, color={0,0,0}));
    connect(LPT2.shaft_b, generator.shaft)
      annotation (Line(points={{218,34},{255.9,33.9}}, color={0,0,0}));
    connect(tee2.port_1, LPT2.portHP) annotation (Line(points={{188,50},{192,50},{
            192,40},{198,40}}, color={0,127,255}));
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-16}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,6}}, color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,-14},{
            178,-64},{40,-64}}, color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{276,100},{276,-4},{186,-4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portLP, tee2.port_2) annotation (Line(points={{138,40},{154,40},
            {154,50},{168,50}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=86400,
        Interval=30,
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end HTGR_Rankine_Cycle_Transient_JY_v1_step3_comp;

  model HTGR_Rankine_Cycle_Transient_JY_v1_step2_comp
    "Without Moister separators"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable ControlSystems.CS_Rankine_Xe100_Based_Secondary CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{78,120},{98,140}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{256,24},{276,44}})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{246,-62},{226,-42}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-30},{-44,-50}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
    TRANSFORM.Fluid.Sensors.Pressure     sensor_p(redeclare package Medium =
          Modelica.Media.Water.StandardWater, redeclare function iconUnit =
          TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar)
                                                         annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-18,76})));
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
          origin={-62,40})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));
    Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
      annotation (Placement(transformation(extent={{-100,106},{-88,120}})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT1(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
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
          origin={-4,-40})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-34})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-70,-40})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-74},{20,-54}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-23})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,-50},{-90,-30}})));
    TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
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
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,-4})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{116,120},{136,140}})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{300,100},{300,-78},{-34,-78},{-34,-47}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-24,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-80,100},{-80,112},{-84,112},{-84,113},{-87.4,113}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(points={{-10,-40},{-24,-40}},
                                      color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-24}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-60,-40},{-44,-40}},color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-118,100},{-118,-62},{-70,-62},{-70,-43.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{236,-62},
            {236,-64},{40,-64}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-64},{16,-64},
            {16,-40},{2,-40}},                         color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(actuatorBus.Divert_Valve_Position, LPT1_Bypass.opening) annotation (
        Line(
        points={{30,100},{288,100},{288,-34},{102,-34}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-30},{242,-42},{243,-42}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary.ports[1]) annotation (Line(points={{-82,72},{-96,
            72}},                                      color={0,127,255}));
    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,40},{-46,40},
            {-46,72},{-66,72}}, color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,40},{-12,40}}, color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,40},{-34,40},
            {-34,62},{-18,62},{-18,66}}, color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-100,40},{-68,40}}, color={0,127,255}));
    connect(sensor_T2.port_b, port_b)
      annotation (Line(points={{-80,-40},{-100,-40}}, color={0,127,255}));
    connect(TBV.opening, actuatorBus.TBV) annotation (Line(points={{-74,78.4},{-74,
            100},{30,100}},       color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1_Bypass.port_b, pump1.port_a)
      annotation (Line(points={{94,-44},{94,-64},{40,-64}}, color={0,127,255}));
    connect(LPT1.shaft_b, LPT2.shaft_a)
      annotation (Line(points={{138,34},{198,34}}, color={0,0,0}));
    connect(LPT2.shaft_b, generator.shaft)
      annotation (Line(points={{218,34},{255.9,33.9}}, color={0,0,0}));
    connect(tee2.port_1, LPT2.portHP) annotation (Line(points={{188,50},{192,50},{
            192,40},{198,40}}, color={0,127,255}));
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-16}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,6}}, color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,-14},{
            178,-64},{40,-64}}, color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{276,100},{276,-4},{186,-4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1.portLP, tee2.port_2) annotation (Line(points={{138,40},{154,40},
            {154,50},{168,50}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=86400,
        Interval=30,
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end HTGR_Rankine_Cycle_Transient_JY_v1_step2_comp;

  model HTGR_Rankine_Cycle_Transient_JY_v1_step4_comp
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable ControlSystems.CS_Rankine_Xe100_Based_Secondary CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{78,120},{98,140}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{256,24},{276,44}})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{246,-80},{226,-60}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
    TRANSFORM.Fluid.Sensors.Pressure     sensor_p(redeclare package Medium =
          Modelica.Media.Water.StandardWater, redeclare function iconUnit =
          TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar)
                                                         annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-18,76})));
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
          origin={-62,40})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));
    Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
      annotation (Placement(transformation(extent={{-100,106},{-88,120}})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT1(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-40})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-70,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-41})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,-68},{-90,-48}})));
    TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
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
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{116,120},{136,140}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{300,100},{300,-96},{-34,-96},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-24,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-80,100},{-80,112},{-84,112},{-84,113},{-87.4,113}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(points={{-10,-58},{
            -24,-58}},                color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-30}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-60,-58},{-44,-58}},color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-118,100},{-118,-80},{-70,-80},{-70,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{236,-80},
            {236,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(actuatorBus.Divert_Valve_Position, LPT1_Bypass.opening) annotation (
        Line(
        points={{30,100},{288,100},{288,-40},{102,-40}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-48},{242,-60},{243,-60}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary.ports[1]) annotation (Line(points={{-82,72},{-96,
            72}},                                      color={0,127,255}));
    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,40},{-46,40},
            {-46,72},{-66,72}}, color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,40},{-12,40}}, color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,40},{-34,40},
            {-34,62},{-18,62},{-18,66}}, color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-100,40},{-68,40}}, color={0,127,255}));
    connect(sensor_T2.port_b, port_b)
      annotation (Line(points={{-80,-58},{-100,-58}}, color={0,127,255}));
    connect(TBV.opening, actuatorBus.TBV) annotation (Line(points={{-74,78.4},{-74,
            100},{30,100}},       color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.shaft_b, LPT2.shaft_a)
      annotation (Line(points={{138,34},{198,34}}, color={0,0,0}));
    connect(LPT2.shaft_b, generator.shaft)
      annotation (Line(points={{218,34},{255.9,33.9}}, color={0,0,0}));
    connect(tee2.port_1, LPT2.portHP) annotation (Line(points={{188,50},{192,50},{
            192,40},{198,40}}, color={0,127,255}));
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-34}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{276,100},{276,16},{186,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-14},{148,-14},{148,-58},{2,-58}}, color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, volume1.port_a)
      annotation (Line(points={{94,-50},{94,-58},{2,-58}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=86400,
        Interval=30,
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end HTGR_Rankine_Cycle_Transient_JY_v1_step4_comp;

  model HTGR_Rankine_Cycle_Transient_JY_v1_step6
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable ControlSystems.CS_Rankine_Xe100_Based_Secondary CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{78,120},{98,140}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{256,24},{276,44}})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{246,-80},{226,-60}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
    TRANSFORM.Fluid.Sensors.Pressure     sensor_p(redeclare package Medium =
          Modelica.Media.Water.StandardWater, redeclare function iconUnit =
          TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar)
                                                         annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-18,76})));
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
          origin={-62,40})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));
    Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
      annotation (Placement(transformation(extent={{-100,106},{-88,120}})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT1(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-24})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-70,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-41})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,-68},{-90,-48}})));
    TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
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
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{116,120},{136,140}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{24,-50},{44,-30}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={68,-40})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{300,100},{300,-96},{-34,-96},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-24,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-80,100},{-80,112},{-84,112},{-84,113},{-87.4,113}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(points={{-10,-58},{
            -24,-58}},                color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-14}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-60,-58},{-44,-58}},color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-118,100},{-118,-80},{-70,-80},{-70,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{236,-80},
            {236,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-48},{242,-60},{243,-60}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary.ports[1]) annotation (Line(points={{-82,72},{-96,
            72}},                                      color={0,127,255}));
    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,40},{-46,40},
            {-46,72},{-66,72}}, color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,40},{-12,40}}, color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,40},{-34,40},
            {-34,62},{-18,62},{-18,66}}, color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-100,40},{-68,40}}, color={0,127,255}));
    connect(sensor_T2.port_b, port_b)
      annotation (Line(points={{-80,-58},{-100,-58}}, color={0,127,255}));
    connect(TBV.opening, actuatorBus.TBV) annotation (Line(points={{-74,78.4},{-74,
            100},{30,100}},       color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.shaft_b, LPT2.shaft_a)
      annotation (Line(points={{138,34},{198,34}}, color={0,0,0}));
    connect(LPT2.shaft_b, generator.shaft)
      annotation (Line(points={{218,34},{255.9,33.9}}, color={0,0,0}));
    connect(tee2.port_1, LPT2.portHP) annotation (Line(points={{188,50},{192,50},{
            192,40},{198,40}}, color={0,127,255}));
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-34}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{276,100},{276,16},{186,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-14},{148,-14},{148,-58},{2,-58}}, color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={
            {94,-34},{94,-40},{78,-40}}, color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary1.ports[1])
      annotation (Line(points={{58,-40},{44,-40}}, color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-154,100},{-154,-54},{68,-54},{68,-43.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.openingLPTv, LPT1_Bypass.opening) annotation (Line(
        points={{30,100},{30,20},{116,20},{116,-24},{102,-24}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=86400,
        Interval=30,
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end HTGR_Rankine_Cycle_Transient_JY_v1_step6;

  model HTGR_Rankine_Cycle_Transient_JY_v1_step5_comp
    "1st extracted steam goes to boundary"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable ControlSystems.CS_Rankine_Xe100_Based_Secondary CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{78,120},{98,140}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{256,24},{276,44}})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{246,-80},{226,-60}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
    TRANSFORM.Fluid.Sensors.Pressure     sensor_p(redeclare package Medium =
          Modelica.Media.Water.StandardWater, redeclare function iconUnit =
          TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar)
                                                         annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-18,76})));
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
          origin={-62,40})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));
    Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
      annotation (Placement(transformation(extent={{-100,106},{-88,120}})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT1(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-70,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-41})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-110,-68},{-90,-48}})));
    TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
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
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{116,120},{136,140}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{16,-44},{36,-24}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={60,-34})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{300,100},{300,-96},{-34,-96},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-24,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-80,100},{-80,112},{-84,112},{-84,113},{-87.4,113}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(points={{-10,-58},{
            -24,-58}},                color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-60,-58},{-44,-58}},color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-118,100},{-118,-80},{-70,-80},{-70,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{236,-80},
            {236,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(actuatorBus.Divert_Valve_Position, LPT1_Bypass.opening) annotation (
        Line(
        points={{30,100},{288,100},{288,-16},{102,-16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-48},{242,-60},{243,-60}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,40},{-46,40},
            {-46,72},{-66,72}}, color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,40},{-12,40}}, color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,40},{-34,40},
            {-34,62},{-18,62},{-18,66}}, color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-100,40},{-68,40}}, color={0,127,255}));
    connect(sensor_T2.port_b, port_b)
      annotation (Line(points={{-80,-58},{-100,-58}}, color={0,127,255}));
    connect(TBV.opening, actuatorBus.TBV) annotation (Line(points={{-74,78.4},{-74,
            100},{30,100}},       color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.shaft_b, LPT2.shaft_a)
      annotation (Line(points={{138,34},{198,34}}, color={0,0,0}));
    connect(LPT2.shaft_b, generator.shaft)
      annotation (Line(points={{218,34},{255.9,33.9}}, color={0,0,0}));
    connect(tee2.port_1, LPT2.portHP) annotation (Line(points={{188,50},{192,50},{
            192,40},{198,40}}, color={0,127,255}));
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-34}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{276,100},{276,16},{186,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-14},{148,-14},{148,-58},{2,-58}}, color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={
            {94,-26},{94,-34},{70,-34}}, color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{50,-34},{36,-34}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=86400,
        Interval=30,
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end HTGR_Rankine_Cycle_Transient_JY_v1_step5_comp;

  model HTGR_Rankine_Cycle_Transient_JY_v1_step7_comp
    "1st extracted steam goes to boundary"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable
        ControlSystems.CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine
        CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{64,122},{84,142}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={238,66})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{246,-80},{226,-60}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
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
          origin={-62,40})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
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
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=30)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-70,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-41})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-150,30},{-130,50}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-150,-68},{-130,-48}})));
    TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
      annotation (Placement(transformation(extent={{130,-10},{150,10}}),
          iconTransformation(extent={{130,-10},{150,10}})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
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
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{88,122},{108,142}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-104,-42},{-84,-22}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={44,-32})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{258,100},{258,-98},{-34,-98},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-20,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-30,76},{-80,76},{-80,112},{-85,112}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(points={{-10,-58},{
            -24,-58}},                color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-60,-58},{-44,-58}},color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-118,100},{-118,-80},{-70,-80},{-70,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{236,-80},
            {236,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-48},{242,-60},{243,-60}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,40},{-46,40},
            {-46,72},{-66,72}}, color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,40},{-12,40}}, color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,40},{
            -34,40},{-34,62},{-14,62},{-14,66}},
                                         color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-140,40},{-68,40}}, color={0,127,255}));
    connect(sensor_T2.port_b, port_b)
      annotation (Line(points={{-80,-58},{-140,-58}}, color={0,127,255}));
    connect(TBV.opening, actuatorBus.TBV) annotation (Line(points={{-74,78.4},{-74,
            100},{30,100}},       color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.shaft_b, LPT2.shaft_a)
      annotation (Line(points={{138,34},{198,34}}, color={0,0,0}));
    connect(tee2.port_1, LPT2.portHP) annotation (Line(points={{188,50},{192,50},{
            192,40},{198,40}}, color={0,127,255}));
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-34}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{248,100},{248,16},{186,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-58},{2,-58}},                     color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={{94,-26},
            {94,-32},{54,-32}},          color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{34,-32},{-84,-32}},color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-30,-42},{44,-42},{44,-35.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(actuatorBus.openingLPTv, LPT1_Bypass.opening) annotation (Line(
        points={{30,100},{30,20},{116,20},{116,-16},{102,-16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(generator.shaft, LPT2.shaft_b) annotation (Line(points={{238.1,55.9},
            {238.1,34},{218,34}}, color={0,0,0}));
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
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end HTGR_Rankine_Cycle_Transient_JY_v1_step7_comp;

  model HTGR_Rankine_Cycle_Transient_JY_v1_step8_comp
    "Bypass massflow is controlled by Power setpoint"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable
        ControlSystems.CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine
        CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{64,122},{84,142}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={238,66})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{246,-80},{226,-60}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
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
          origin={-62,40})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
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
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=30)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-70,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-41})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-150,30},{-130,50}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-150,-68},{-130,-48}})));
    TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
      annotation (Placement(transformation(extent={{130,-10},{150,10}}),
          iconTransformation(extent={{130,-10},{150,10}})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
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
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{88,122},{108,142}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-104,-42},{-84,-22}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={44,-32})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{258,100},{258,-98},{-34,-98},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-20,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-30,76},{-80,76},{-80,112},{-85,112}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(points={{-10,-58},{
            -24,-58}},                color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-60,-58},{-44,-58}},color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-118,100},{-118,-80},{-70,-80},{-70,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{236,-80},
            {236,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-48},{242,-60},{243,-60}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,40},{-46,40},
            {-46,72},{-66,72}}, color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,40},{-12,40}}, color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,40},{
            -34,40},{-34,62},{-14,62},{-14,66}},
                                         color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-140,40},{-68,40}}, color={0,127,255}));
    connect(sensor_T2.port_b, port_b)
      annotation (Line(points={{-80,-58},{-140,-58}}, color={0,127,255}));
    connect(TBV.opening, actuatorBus.TBV) annotation (Line(points={{-74,78.4},{-74,
            100},{30,100}},       color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.shaft_b, LPT2.shaft_a)
      annotation (Line(points={{138,34},{198,34}}, color={0,0,0}));
    connect(tee2.port_1, LPT2.portHP) annotation (Line(points={{188,50},{192,50},{
            192,40},{198,40}}, color={0,127,255}));
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-34}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{248,100},{248,16},{186,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-58},{2,-58}},                     color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={{94,-26},
            {94,-32},{54,-32}},          color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{34,-32},{-84,-32}},color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-30,-42},{44,-42},{44,-35.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(actuatorBus.openingLPTv, LPT1_Bypass.opening) annotation (Line(
        points={{30,100},{30,20},{116,20},{116,-16},{102,-16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(generator.shaft, LPT2.shaft_b) annotation (Line(points={{238.1,55.9},
            {238.1,34},{218,34}}, color={0,0,0}));
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
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end HTGR_Rankine_Cycle_Transient_JY_v1_step8_comp;

  model HTGR_Rankine_Cycle_Transient_JY_v1_step9 "Pressure Control Trial"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable
        ControlSystems.CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl
        CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{64,122},{84,142}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={238,66})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{244,-62},{224,-42}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
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
          origin={-62,40})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
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
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=30)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-70,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-19})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-150,30},{-130,50}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-150,-68},{-130,-48}})));
    TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
      annotation (Placement(transformation(extent={{130,-10},{150,10}}),
          iconTransformation(extent={{130,-10},{150,10}})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
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
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{88,122},{108,142}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-104,-42},{-84,-22}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={44,-32})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{258,100},{258,-98},{-34,-98},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-20,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-30,76},{-80,76},{-80,112},{-85,112}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(points={{-10,-58},{
            -24,-58}},                color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-60,-58},{-44,-58}},color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-118,100},{-118,-80},{-70,-80},{-70,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{234,-62},
            {234,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-26},{242,-42},{241,-42}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,40},{-46,40},
            {-46,72},{-66,72}}, color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,40},{-12,40}}, color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,40},{
            -34,40},{-34,62},{-14,62},{-14,66}},
                                         color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-140,40},{-68,40}}, color={0,127,255}));
    connect(sensor_T2.port_b, port_b)
      annotation (Line(points={{-80,-58},{-140,-58}}, color={0,127,255}));
    connect(TBV.opening, actuatorBus.TBV) annotation (Line(points={{-74,78.4},{-74,
            100},{30,100}},       color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.shaft_b, LPT2.shaft_a)
      annotation (Line(points={{138,34},{198,34}}, color={0,0,0}));
    connect(tee2.port_1, LPT2.portHP) annotation (Line(points={{188,50},{192,50},{
            192,40},{198,40}}, color={0,127,255}));
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-12}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{248,100},{248,16},{186,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-58},{2,-58}},                     color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={{94,-26},
            {94,-32},{54,-32}},          color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{34,-32},{-84,-32}},color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-30,-42},{44,-42},{44,-35.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(actuatorBus.openingLPTv, LPT1_Bypass.opening) annotation (Line(
        points={{30,100},{30,20},{116,20},{116,-16},{102,-16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(generator.shaft, LPT2.shaft_b) annotation (Line(points={{238.1,55.9},
            {238.1,34},{218,34}}, color={0,0,0}));
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
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end HTGR_Rankine_Cycle_Transient_JY_v1_step9;

  model HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_comp
    "Pressure Control done!!!"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable
        ControlSystems.CS_threeStagedTurbine_HTGR
        CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);
    Real time_altered;
    Real time_initialization = 7e4;
    Real electricity_generation_Norm;
    Real electricity_demand_Norm;
    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{64,122},{84,142}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={238,66})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{244,-62},{224,-42}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled FWCP(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
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
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
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
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=30)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-108,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-19})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
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
      V=5,
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
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{90,122},{110,142}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-104,-42},{-84,-22}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={28,-32})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T3(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-6,6},{6,-6}},
          rotation=180,
          origin={66,-32})));
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
        points={{30,100},{258,100},{258,-98},{-34,-98},{-34,-65}},
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
    connect(volume1.port_b,FWCP. port_a) annotation (Line(points={{-10,-58},{
            -24,-58}},                color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-30,-44},{-56,-44},{-56,-74},{-108,-74},{-108,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{234,-62},
            {234,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-26},{242,-42},{241,-42}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
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
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-12}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{248,100},{248,16},{186,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-58},{2,-58}},                     color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{18,-32},{-84,-32}},color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-30,-42},{28,-42},{28,-35.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(actuatorBus.openingLPTv, LPT1_Bypass.opening) annotation (Line(
        points={{30,100},{30,20},{116,20},{116,-16},{102,-16}},
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
    connect(LPT1_Bypass.port_b, sensor_T3.port_a) annotation (Line(points={{94,
            -26},{94,-32},{72,-32}}, color={0,127,255}));
    connect(sensor_T3.port_b, sensor_m_flow.port_a)
      annotation (Line(points={{60,-32},{38,-32}}, color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-140,48},{-68,48}}, color={0,127,255}));
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
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_comp;

  model HTGR_Rankine_Cycle_Transient_JY_v1_step10_TBV_Control
    "Pressure Control Trial"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable
        ControlSystems.CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TEST_TBVcontrol
        CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{64,122},{84,142}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={238,66})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{244,-62},{224,-42}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
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
          origin={-62,40})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
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
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=30)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-70,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-19})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-150,30},{-130,50}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-150,-68},{-130,-48}})));
    TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
      annotation (Placement(transformation(extent={{130,-10},{150,10}}),
          iconTransformation(extent={{130,-10},{150,10}})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
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
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{88,122},{108,142}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-104,-42},{-84,-22}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={44,-32})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{258,100},{258,-98},{-34,-98},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-20,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-30,76},{-80,76},{-80,112},{-85,112}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(points={{-10,-58},{
            -24,-58}},                color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-60,-58},{-44,-58}},color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-118,100},{-118,-80},{-70,-80},{-70,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{234,-62},
            {234,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-26},{242,-42},{241,-42}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,40},{-46,40},
            {-46,72},{-66,72}}, color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,40},{-12,40}}, color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,40},{
            -34,40},{-34,62},{-14,62},{-14,66}},
                                         color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-140,40},{-68,40}}, color={0,127,255}));
    connect(sensor_T2.port_b, port_b)
      annotation (Line(points={{-80,-58},{-140,-58}}, color={0,127,255}));
    connect(TBV.opening, actuatorBus.TBV) annotation (Line(points={{-74,78.4},{-74,
            100},{30,100}},       color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.shaft_b, LPT2.shaft_a)
      annotation (Line(points={{138,34},{198,34}}, color={0,0,0}));
    connect(tee2.port_1, LPT2.portHP) annotation (Line(points={{188,50},{192,50},{
            192,40},{198,40}}, color={0,127,255}));
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-12}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{248,100},{248,16},{186,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-58},{2,-58}},                     color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={{94,-26},
            {94,-32},{54,-32}},          color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{34,-32},{-84,-32}},color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-30,-42},{44,-42},{44,-35.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(actuatorBus.openingLPTv, LPT1_Bypass.opening) annotation (Line(
        points={{30,100},{30,20},{116,20},{116,-16},{102,-16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(generator.shaft, LPT2.shaft_b) annotation (Line(points={{238.1,55.9},
            {238.1,34},{218,34}}, color={0,0,0}));
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
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end HTGR_Rankine_Cycle_Transient_JY_v1_step10_TBV_Control;

  model
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type1
    "Pressure Control done!!!"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable
        ControlSystems.CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TCVcontrol_CompDegradation_type1
        CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{64,122},{84,142}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={238,66})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{244,-62},{224,-42}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled FWP(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
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
          origin={-62,40})));

    TRANSFORM.Fluid.Valves.ValveLinear TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
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
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
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
          origin={48,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=30)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-104,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{92,-92},{72,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-19})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-150,30},{-130,50}})));
    TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-150,-68},{-130,-48}})));
    TRANSFORM.Electrical.Interfaces.ElectricalPowerPort_Flow port_e
      annotation (Placement(transformation(extent={{250,-10},{270,10}}),
          iconTransformation(extent={{250,-10},{270,10}})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
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
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{90,122},{110,142}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-104,-42},{-84,-22}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={44,-32})));
    Modelica.Blocks.Sources.Ramp     TBV_open(
      height=0,
      duration=1000,
      offset=0,
      startTime=100000)
      annotation (Placement(transformation(extent={{-56,84},{-66,94}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled FWP1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-62,-48},{-82,-68}})));
    Component_Degradation.PumpMotor_degradation_type1 pumpMotor_degradation(
      start_time=2e5,
      r=0.9,
      s_nom=1200,
      s_min=600)  annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-72,-82})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=FWP.m_flow)
      annotation (Placement(transformation(extent={{-46,-116},{-66,-96}})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, FWP.inputSignal) annotation (Line(
        points={{30,100},{258,100},{258,-98},{-34,-98},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-20,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-30,118},{-80,118},{-80,112},{-85,112}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-30,-42},{-82,-42},{-82,-46},{-88,-46},{-88,-74},{
            -104,-74},{-104,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{234,-62},
            {234,-82},{92,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{72,-82},{60,
            -82},{60,-58},{54,-58}},                   color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-26},{242,-42},{241,-42}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
    connect(actuatorBus.opening_TCV, TCV.opening) annotation (Line(
        points={{30.1,100.1},{-4,100.1},{-4,46.4}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,40},{-46,40},
            {-46,72},{-66,72}}, color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,40},{-12,40}}, color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,40},{
            -34,40},{-34,62},{-14,62},{-14,66}},
                                         color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-140,40},{-68,40}}, color={0,127,255}));
    connect(LPT1.shaft_b, LPT2.shaft_a)
      annotation (Line(points={{138,34},{198,34}}, color={0,0,0}));
    connect(tee2.port_1, LPT2.portHP) annotation (Line(points={{188,50},{192,50},{
            192,40},{198,40}}, color={0,127,255}));
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-12}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{92,-82}},color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{248,100},{248,16},{186,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-58},{54,-58}},                    color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={{94,-26},
            {94,-32},{54,-32}},          color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{34,-32},{-84,-32}},color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-162,100},{-162,-46},{44,-46},{44,-35.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(actuatorBus.openingLPTv, LPT1_Bypass.opening) annotation (Line(
        points={{30,100},{30,20},{116,20},{116,-16},{102,-16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(generator.shaft, LPT2.shaft_b) annotation (Line(points={{238.1,55.9},
            {238.1,34},{218,34}}, color={0,0,0}));
    connect(TBV_open.y, TBV.opening) annotation (Line(points={{-66.5,89},{-74,
            89},{-74,78.4}}, color={0,0,127}));
    connect(FWP.port_b, FWP1.port_a)
      annotation (Line(points={{-44,-58},{-62,-58}}, color={0,127,255}));
    connect(pumpMotor_degradation.s_out, FWP1.inputSignal)
      annotation (Line(points={{-72,-73},{-72,-65}}, color={0,0,127}));
    connect(realExpression.y, pumpMotor_degradation.m_flow) annotation (Line(
          points={{-67,-106},{-72,-106},{-72,-90.8}}, color={0,0,127}));
    connect(FWP1.port_b, sensor_T2.port_a)
      annotation (Line(points={{-82,-58},{-94,-58}}, color={0,127,255}));
    connect(sensor_T2.port_b, port_b)
      annotation (Line(points={{-114,-58},{-140,-58}}, color={0,127,255}));
    connect(volume1.port_b, FWP.port_a)
      annotation (Line(points={{42,-58},{-24,-58}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,
              -100},{260,140}}),                                  graphics={
          Rectangle(
            extent={{-2.09756,2},{83.9024,-2}},
            lineColor={0,0,0},
            origin={-47.9024,-60},
            rotation=360,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-1.81329,5},{66.1867,-5}},
            lineColor={0,0,0},
            origin={-70.1867,-37},
            rotation=0,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-16,3},{16,-3}},
            lineColor={0,0,0},
            fillColor={66,200,200},
            fillPattern=FillPattern.HorizontalCylinder,
            origin={2,34},
            rotation=-90),
          Rectangle(
            extent={{-1.81332,3},{66.1869,-3}},
            lineColor={0,0,0},
            origin={-20.1867,1},
            rotation=0,
            fillColor={135,135,135},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-72,50},{-24,38}},
            lineColor={0,0,0},
            fillColor={66,200,200},
            fillPattern=FillPattern.HorizontalCylinder),
          Polygon(
            points={{-2,20},{-2,-10},{28,-28},{28,40},{-2,20}},
            lineColor={0,0,0},
            fillColor={0,114,208},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{9,-4},{19,10}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="T"),
          Ellipse(
            extent={{44,16},{72,-10}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-0.4,3},{15.5,-3}},
            lineColor={0,0,0},
            origin={28.4272,-25},
            rotation=0,
            fillColor={0,128,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-0.43805,2.7864},{15.9886,-2.7864}},
            lineColor={0,0,0},
            origin={43.2136,-37.989},
            rotation=90,
            fillColor={0,128,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(
            extent={{30,-38},{58,-64}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-0.373344,2},{13.6267,-2}},
            lineColor={0,0,0},
            origin={16.3733,-52},
            rotation=0,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-0.487802,2},{19.5122,-2}},
            lineColor={0,0,0},
            origin={18,-34.488},
            rotation=-90,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-0.243902,2},{9.7562,-2}},
            lineColor={0,0,0},
            origin={-48,-58.244},
            rotation=-90,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Rectangle(
            extent={{-0.578156,2.1722},{23.1262,-2.1722}},
            lineColor={0,0,0},
            origin={19.4218,-35.828},
            rotation=180,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(
            extent={{-6,-30},{6,-42}},
            lineColor={0,0,0},
            fillPattern=FillPattern.Sphere,
            fillColor={0,100,199}),
          Polygon(
            points={{-4,-40},{-8,-44},{8,-44},{4,-40},{-4,-40}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={0,0,0},
            fillPattern=FillPattern.VerticalCylinder),
          Rectangle(
            extent={{-22,50},{4,38}},
            lineColor={0,0,0},
            fillColor={66,200,200},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(
            extent={{-32,53},{-14,35}},
            lineColor={95,95,95},
            fillColor={175,175,175},
            fillPattern=FillPattern.Sphere),
          Rectangle(
            extent={{-22,53},{-24,65}},
            lineColor={0,0,0},
            fillColor={95,95,95},
            fillPattern=FillPattern.VerticalCylinder),
          Rectangle(
            extent={{-32,67},{-14,65}},
            lineColor={0,0,0},
            fillColor={181,0,0},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(
            extent={{-21,53},{-25,35}},
            lineColor={0,0,0},
            fillPattern=FillPattern.VerticalCylinder,
            fillColor={162,162,0}),
          Text(
            extent={{53,-6},{63,8}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="G"),
          Text(
            extent={{39,-58},{49,-44}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="C"),
          Polygon(
            points={{1,-33},{1,-39},{-3,-36},{1,-33}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={255,255,255})}),                            Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},{260,
              140}})),
      experiment(
        StopTime=86400,
        Interval=30,
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type1;

  package Component_Degradation
    model PumpMotor_degradation_type1
      parameter Modelica.Units.SI.Time start_time "Time degradation begins";
      Modelica.Units.SI.Time linear_end_time "Time degradation would end with no feedback";
      Real tau "Mass flow rate feedback to degradation rate";

      parameter Real r "randomness parameter";
    //  parameter Modelica.Units.SI.MassFlowRate massflow_nom "nominal massflow";

      parameter Modelica.Units.SI.AngularVelocity s_nom "norminal shaft rotational speed";
      parameter Modelica.Units.SI.AngularVelocity s_min "minimum permissible shaft rotational speed";
      Modelica.Units.SI.AngularAcceleration sdot_linear "Nominal rate of linear speed decrease. Must be negative";
      Modelica.Units.SI.AngularAcceleration sdot "Rate of change of shaft speed";

      Modelica.Blocks.Interfaces.RealOutput s_out annotation (Placement(
            transformation(extent={{80,-10},{100,10}}), iconTransformation(extent={{
                80,-10},{100,10}})));
      Modelica.Blocks.Interfaces.RealInput m_flow annotation (Placement(
            transformation(extent={{-100,-12},{-76,12}}), iconTransformation(extent={{-100,
                -12},{-76,12}})));
    equation
      linear_end_time = 400420;
      sdot_linear = (s_min - s_nom) / (linear_end_time - start_time);
      //tau = m_flow*r / massflow_nom;
      tau = r;
      sdot = sdot_linear*tau;

      if time < start_time then
        s_out = s_nom;
      elseif time >=start_time  and s_out >=s_min then
        s_out = sdot*(time-start_time) + s_nom;
      else
        s_out = s_min;
      end if
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
      annotation (Icon(graphics={Rectangle(extent={{-92,100},{92,-100}},
                lineColor={0,0,0}), Text(
              extent={{-74,56},{74,-60}},
              textColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="%Degradation
Model")}));
    end PumpMotor_degradation_type1;

    model PumpMotor_degradation_type2
      parameter Modelica.Units.SI.Time start_time "Time degradation begins";
      Modelica.Units.SI.Time linear_end_time "Time degradation would end with no feedback";
      Real tau "Mass flow rate feedback to degradation rate";

      parameter Real r "randomness parameter";
      parameter Modelica.Units.SI.MassFlowRate massflow_nom "nominal massflow";

      parameter Modelica.Units.SI.AngularVelocity s_nom "norminal shaft rotational speed";
      parameter Modelica.Units.SI.AngularVelocity s_min "minimum permissible shaft rotational speed";
      Modelica.Units.SI.AngularAcceleration sdot_linear "Nominal rate of linear speed decrease. Must be negative";
      Modelica.Units.SI.AngularAcceleration sdot "Rate of change of shaft speed";

      Modelica.Blocks.Interfaces.RealOutput s_out annotation (Placement(
            transformation(extent={{80,-10},{100,10}}), iconTransformation(extent={{
                80,-10},{100,10}})));
      Modelica.Blocks.Interfaces.RealInput m_flow annotation (Placement(
            transformation(extent={{-100,-12},{-76,12}}), iconTransformation(extent={{-100,
                -12},{-76,12}})));
    equation
      linear_end_time = 400420;
      sdot_linear = (s_min - s_nom) / (linear_end_time - start_time);
      tau = m_flow*r / massflow_nom;
      sdot = sdot_linear*tau;

      if time < start_time then
        s_out = s_nom;
      elseif time >=start_time  and s_out >=s_min then
        s_out = sdot*(time-start_time) + s_nom;
      else
        s_out = s_min;
      end if
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
      annotation (Icon(graphics={Rectangle(extent={{-92,100},{92,-100}},
                lineColor={0,0,0}), Text(
              extent={{-74,56},{74,-60}},
              textColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="%Degradation
Model")}));
    end PumpMotor_degradation_type2;

    model valve_degradation
      Modelica.Blocks.Interfaces.RealOutput open_out annotation (Placement(
            transformation(extent={{80,-10},{100,10}}), iconTransformation(extent={{
                80,-10},{100,10}})));
      Modelica.Blocks.Interfaces.RealInput open_in annotation (Placement(
            transformation(extent={{-100,-12},{-76,12}}), iconTransformation(extent={{-100,
                -12},{-76,12}})));
      HazardFunctionTable hazardFunctionTable
        annotation (Placement(transformation(extent={{-96,78},{-76,98}})));
    equation
      open_out = (1- hazardFunctionTable.CumulHazardFunction.y[2])*open_in
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));

      annotation (Icon(graphics={Rectangle(extent={{-92,100},{92,-100}},
                lineColor={0,0,0}), Text(
              extent={{-74,56},{74,-60}},
              textColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="%Aging
Model")}));
    end valve_degradation;

    model valveFail
      Modelica.Blocks.Interfaces.RealOutput failureIndex annotation (Placement(
            transformation(extent={{80,10},{100,30}}),  iconTransformation(extent={{80,10},
                {100,30}})));
      Modelica.Blocks.Interfaces.RealInput hazard annotation (Placement(
            transformation(extent={{-100,34},{-76,58}}), iconTransformation(extent={
                {-100,34},{-76,58}})));
      Modelica.Blocks.Interfaces.RealInput random annotation (Placement(
            transformation(extent={{-100,-58},{-76,-34}}), iconTransformation(
              extent={{-100,-54},{-76,-30}})));

      Modelica.Blocks.Interfaces.BooleanOutput failreBooleanIndex
        annotation (Placement(transformation(extent={{80,-30},{100,-10}})));
    initial equation
      failreBooleanIndex = false;

    equation
      failureIndex = random - hazard  annotation (Icon(
            coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(
              preserveAspectRatio=false)));
      if failreBooleanIndex then
        failreBooleanIndex = true;
      else
        if failureIndex < 0 then
          failreBooleanIndex = true;
        else
          failreBooleanIndex = false;
        end if;
      end if;
      annotation (Icon(graphics={Rectangle(extent={{-100,100},{80,-100}},
                lineColor={0,0,0}), Text(
              extent={{-74,56},{74,-60}},
              textColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="Failure")}));
    end valveFail;

    model valveFail_Index
      HazardFunctionTable hazardFunctionTable
        annotation (Placement(transformation(extent={{-72,30},{-54,48}})));
      Modelica.Blocks.Noise.UniformNoise uniformNoise(
        samplePeriod=1000,
        y_min=0,
        y_max=1)
        annotation (Placement(transformation(extent={{-72,-36},{-56,-20}})));
      valveFail Index
        annotation (Placement(transformation(extent={{4,-10},{24,10}})));
      Modelica.Blocks.Interfaces.RealOutput failureIndex annotation (Placement(
            transformation(extent={{80,28},{100,48}}),  iconTransformation(extent={{80,20},
                {100,40}})));
      inner Modelica.Blocks.Noise.GlobalSeed globalSeed
        annotation (Placement(transformation(extent={{-94,74},{-74,94}})));
      Modelica.Blocks.Interfaces.BooleanOutput failreIndex_Bool annotation (
          Placement(transformation(extent={{80,-40},{100,-20}}),
            iconTransformation(extent={{80,-40},{100,-20}})));
      Modelica.Blocks.Interfaces.RealInput valve_dervInput annotation (
          Placement(transformation(extent={{-102,-12},{-78,12}}),
            iconTransformation(extent={{-102,-12},{-78,12}})));
      Modelica.Blocks.Math.Product product1
        annotation (Placement(transformation(extent={{-32,14},{-12,34}})));
      Modelica.Blocks.Math.Gain gain(k=1)
        annotation (Placement(transformation(extent={{-32,-38},{-12,-18}})));
      realTobool realTobool1
        annotation (Placement(transformation(extent={{-72,-2},{-54,16}})));
    equation
      connect(Index.failureIndex, failureIndex)
        annotation (Line(points={{23,2},{62,2},{62,38},{90,38}},
                                                  color={0,0,127}));
      connect(Index.failreBooleanIndex, failreIndex_Bool) annotation (Line(
            points={{23,-2},{62,-2},{62,-30},{90,-30}}, color={255,0,255}));
      connect(hazardFunctionTable.HazardValue, product1.u1) annotation (Line(
            points={{-54.9,39},{-44,39},{-44,30},{-34,30}}, color={0,0,127}));
      connect(product1.y, Index.hazard) annotation (Line(points={{-11,24},{-6,
              24},{-6,4.6},{5.2,4.6}}, color={0,0,127}));
      connect(uniformNoise.y, gain.u)
        annotation (Line(points={{-55.2,-28},{-34,-28}}, color={0,0,127}));
      connect(gain.y, Index.random) annotation (Line(points={{-11,-28},{-6,-28},
              {-6,-4.2},{5.2,-4.2}}, color={0,0,127}));
      connect(valve_dervInput, realTobool1.valve_der) annotation (Line(points={
              {-90,0},{-76,0},{-76,7},{-70.92,7}}, color={0,0,127}));
      connect(realTobool1.derBooleanIndex, product1.u2) annotation (Line(points=
             {{-54.9,5.2},{-44,5.2},{-44,18},{-34,18}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
              extent={{-100,100},{80,-100}},
              lineColor={0,0,0},
              lineThickness=0.5), Text(
              extent={{-54,16},{34,-28}},
              textColor={0,0,0},
              textString="%Valve
Failure")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end valveFail_Index;

    model HazardFunctionTable
      Modelica.Blocks.Sources.CombiTimeTable CumulHazardFunction(table=[3600,
            1.76124e-05,1.76124e-05; 2595600,0.000855354,0.000872967; 5187600,
            0.000443919,0.001316885; 7779600,0.000358389,0.001675274; 10371600,
            0.000312179,0.001987453; 12963600,0.000281804,0.002269257; 15555600,
            0.000259763,0.00252902; 18147600,0.000242772,0.002771792; 20739600,
            0.000229126,0.003000918; 23331600,0.000217835,0.003218753; 25923600,
            0.000208279,0.003427032; 28515600,0.000200046,0.003627078; 31107600,
            0.000192851,0.003819929; 33699600,0.000186489,0.004006418; 36291600,
            0.000180807,0.004187225; 38883600,0.00017569,0.004362915; 41475600,
            0.000171047,0.004533962; 44067600,0.000166809,0.004700771; 46659600,
            0.000162919,0.00486369; 49251600,0.00015933,0.005023019; 51843600,
            0.000156004,0.005179023; 54435600,0.000152911,0.005331935; 57027600,
            0.000150024,0.005481958; 59619600,0.00014732,0.005629278; 62211600,
            0.00014478,0.005774059; 64803600,0.000142389,0.005916447; 67395600,
            0.000140131,0.006056579; 69987600,0.000137995,0.006194574; 72579600,
            0.00013597,0.006330544; 75171600,0.000134046,0.00646459; 77763600,
            0.000132215,0.006596804; 80355600,0.000130469,0.006727274; 82947600,
            0.000128803,0.006856077; 85539600,0.000127209,0.006983286; 88131600,
            0.000125683,0.007108969; 90723600,0.00012422,0.00723319; 93315600,
            0.000122816,0.007356006; 95907600,0.000121467,0.007477473; 98499600,
            0.000120168,0.007597641; 101091600,0.000118918,0.007716559;
            103683600,0.000117713,0.007834271; 106275600,0.00011655,0.007950821;
            108867600,0.000115426,0.008066247; 111459600,0.000114341,
            0.008180588; 114051600,0.000113291,0.008293879; 116643600,
            0.000112274,0.008406153; 119235600,0.000111289,0.008517442;
            121827600,0.000110334,0.008627776; 124419600,0.000109408,
            0.008737184; 127011600,0.000108509,0.008845692; 129603600,
            0.000107635,0.008953328; 132195600,0.000106787,0.009060115;
            134787600,0.000105961,0.009166076; 137379600,0.000105158,
            0.009271234; 139971600,0.000104377,0.009375611; 142563600,
            0.000103615,0.009479226; 145155600,0.000102873,0.009582099;
            147747600,0.00010215,0.009684249; 150339600,0.000101445,0.009785694;
            152931600,0.000100756,0.00988645; 155523600,0.000100084,0.009986534;
            158115600,9.94283e-05,0.010085963; 160707600,9.87873e-05,0.01018475;
            163299600,9.81609e-05,0.010282911; 165891600,9.75485e-05,
            0.010380459; 168483600,9.69495e-05,0.010477409; 171075600,
            9.63636e-05,0.010573772; 173667600,9.57901e-05,0.010669562;
            176259600,9.52287e-05,0.010764791; 178851600,9.46789e-05,0.01085947;
            181443600,9.41403e-05,0.01095361; 184035600,9.36126e-05,0.011047223;
            186627600,9.30953e-05,0.011140318; 189219600,9.25882e-05,
            0.011232906; 191811600,9.20908e-05,0.011324997; 194403600,
            9.16029e-05,0.0114166; 196995600,9.11242e-05,0.011507724; 199587600,
            9.06544e-05,0.011598379; 202179600,9.01931e-05,0.011688572;
            204771600,8.97402e-05,0.011778312; 207363600,8.92954e-05,
            0.011867607; 209955600,8.88584e-05,0.011956466; 212547600,
            8.8429e-05,0.012044895; 215139600,8.8007e-05,0.012132902; 217731600,
            8.75922e-05,0.012220494; 220323600,8.71843e-05,0.012307678;
            222915600,8.67832e-05,0.012394462; 225507600,8.63886e-05,0.01248085;
            228099600,8.60005e-05,0.012566851; 230691600,8.56186e-05,
            0.012652469; 233283600,8.52427e-05,0.012737712; 235875600,
            8.48727e-05,0.012822585; 238467600,8.45085e-05,0.012907093;
            241059600,8.41498e-05,0.012991243; 243651600,8.37966e-05,0.01307504;
            246243600,8.34487e-05,0.013158488; 248835600,8.31059e-05,
            0.013241594; 251427600,8.27682e-05,0.013324363; 254019600,
            8.24354e-05,0.013406798; 256611600,8.21074e-05,0.013488905;
            259203600,8.17841e-05,0.013570689; 261795600,8.14653e-05,
            0.013652155; 264387600,8.1151e-05,0.013733306; 266979600,8.0841e-05,
            0.013814147; 269571600,8.05353e-05,0.013894682; 272163600,
            8.02337e-05,0.013974916; 274755600,7.99362e-05,0.014054852;
            277347600,7.96426e-05,0.014134494; 279939600,7.93529e-05,
            0.014213847; 282531600,7.9067e-05,0.014292914; 285123600,
            7.87848e-05,0.014371699; 287715600,7.85062e-05,0.014450205;
            290307600,7.82312e-05,0.014528436; 292899600,7.79596e-05,
            0.014606396; 295491600,7.76914e-05,0.014684087; 298083600,
            7.74266e-05,0.014761514; 300675600,7.7165e-05,0.014838679;
            303267600,7.69065e-05,0.014915586; 305859600,7.66512e-05,
            0.014992237; 308451600,7.6399e-05,0.015068636; 311043600,
            7.61497e-05,0.015144785; 313635600,7.59034e-05,0.015220689;
            316227600,7.566e-05,0.015296349])
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      Modelica.Blocks.Interfaces.RealOutput HazardValue annotation (Placement(
            transformation(extent={{80,-10},{100,10}}), iconTransformation(
              extent={{80,-10},{100,10}})));
    equation
      connect(CumulHazardFunction.y[1], HazardValue)
        annotation (Line(points={{11,0},{90,0}},  color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
              extent={{-100,100},{80,-100}},
              lineColor={0,0,0}), Text(
              extent={{-62,40},{46,-40}},
              textColor={0,0,0},
              textString="%Hazard
Table")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end HazardFunctionTable;

    model systemFail_Index
      valveFail_Index valveFail_Index1
        annotation (Placement(transformation(extent={{-32,-10},{-12,8}})));
      valveFail_Index valveFail_Index2
        annotation (Placement(transformation(extent={{-32,-34},{-12,-16}})));
      valveFail_Index valveFail_Index3
        annotation (Placement(transformation(extent={{-32,16},{-12,34}})));
      Modelica.Blocks.Logical.Or or1
        annotation (Placement(transformation(extent={{22,8},{42,28}})));
      Modelica.Blocks.Logical.Or or2
        annotation (Placement(transformation(extent={{52,-10},{72,10}})));
      Modelica.Blocks.Interfaces.RealInput tcv_dervInput annotation (Placement(
            transformation(extent={{-102,14},{-79,36}}), iconTransformation(
              extent={{-106,30},{-86,50}})));
      Modelica.Blocks.Interfaces.RealInput LTV1_dervInput annotation (Placement(
            transformation(extent={{-100,-12},{-79,10}}), iconTransformation(
              extent={{-108,-10},{-88,10}})));
      Modelica.Blocks.Interfaces.RealInput LTV2_dervInput annotation (Placement(
            transformation(extent={{-100,-36},{-79,-14}}), iconTransformation(
              extent={{-108,-50},{-88,-30}})));
      Modelica.Blocks.Interfaces.BooleanOutput y
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    equation
      connect(valveFail_Index3.failreIndex_Bool, or1.u1) annotation (Line(
            points={{-13,22.3},{12,22.3},{12,18},{20,18}}, color={255,0,255}));
      connect(valveFail_Index1.failreIndex_Bool, or1.u2) annotation (Line(
            points={{-13,-3.7},{14,-3.7},{14,10},{20,10}}, color={255,0,255}));
      connect(or1.y, or2.u1) annotation (Line(points={{43,18},{48,18},{48,0},{
              50,0}}, color={255,0,255}));
      connect(valveFail_Index2.failreIndex_Bool, or2.u2) annotation (Line(
            points={{-13,-27.7},{48,-27.7},{48,-8},{50,-8}}, color={255,0,255}));
      connect(valveFail_Index3.valve_dervInput, tcv_dervInput)
        annotation (Line(points={{-31,25},{-90.5,25}}, color={0,0,127}));
      connect(LTV1_dervInput, valveFail_Index1.valve_dervInput) annotation (
          Line(points={{-89.5,-1},{-89.5,-1},{-31,-1}}, color={0,0,127}));
      connect(LTV2_dervInput, valveFail_Index2.valve_dervInput)
        annotation (Line(points={{-89.5,-25},{-31,-25}}, color={0,0,127}));
      connect(or2.y, y)
        annotation (Line(points={{73,0},{100,0}}, color={255,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              lineThickness=0.5), Text(
              extent={{-64,42},{62,-40}},
              textColor={0,0,0},
              textString="%System
Failure
Index")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end systemFail_Index;

    model realTobool
      Modelica.Blocks.Interfaces.RealOutput valve_derOut annotation (Placement(
            transformation(extent={{80,10},{100,30}}),  iconTransformation(extent={{80,10},
                {100,30}})));
      Modelica.Blocks.Interfaces.RealInput valve_der annotation (Placement(
            transformation(extent={{-100,-12},{-76,12}}), iconTransformation(extent=
               {{-100,-12},{-76,12}})));

      Modelica.Blocks.Interfaces.RealOutput derBooleanIndex
        annotation (Placement(transformation(extent={{80,-30},{100,-10}})));
    initial equation
      derBooleanIndex = 0;

    equation
      valve_derOut = abs(valve_der)        annotation (Icon(
            coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(
              preserveAspectRatio=false)));
      if valve_derOut > 0 then
        derBooleanIndex = 1;
      else
        derBooleanIndex = 0;
      end if;
      annotation (Icon(graphics={Rectangle(extent={{-100,100},{80,-100}},
                lineColor={0,0,0}), Text(
              extent={{-74,56},{74,-60}},
              textColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="%Real Der.
to Boolean")}));
    end realTobool;

    model valveFailProto
      Modelica.Blocks.Interfaces.RealOutput failureIndex annotation (Placement(
            transformation(extent={{80,10},{100,30}}),  iconTransformation(extent={{80,10},
                {100,30}})));
      Modelica.Blocks.Interfaces.RealInput hazard annotation (Placement(
            transformation(extent={{-100,34},{-76,58}}), iconTransformation(extent={
                {-100,34},{-76,58}})));
      Modelica.Blocks.Interfaces.RealInput random annotation (Placement(
            transformation(extent={{-100,-58},{-76,-34}}), iconTransformation(
              extent={{-100,-54},{-76,-30}})));

      Modelica.Blocks.Interfaces.BooleanOutput failreBooleanIndex
        annotation (Placement(transformation(extent={{80,-30},{100,-10}})));
    equation
      failureIndex = random - hazard  annotation (Icon(
            coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(
              preserveAspectRatio=false)));
      if failureIndex < 0 then
        failreBooleanIndex = true;
      else
        failreBooleanIndex = false;
      end if;
      annotation (Icon(graphics={Rectangle(extent={{-100,100},{80,-100}},
                lineColor={0,0,0}), Text(
              extent={{-74,56},{74,-60}},
              textColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="Failure")}));
    end valveFailProto;

    model MOV_failToOperate_Model "Separate_System_Failure_Modeling"
      parameter Integer randomSeed = 1234 "random Seed";
      parameter SI.Time strategyChangeTime "strategy Change Timing";
      parameter SI.Time samplePeriod = 2592000 "2592000 sec = 720 hours";
      Modelica.Blocks.Sources.Constant valvedelay6(k=strategyChangeTime)
        annotation (Placement(transformation(extent={{-70,30},{-50,50}})));
      Modelica.Blocks.Sources.ContinuousClock clock4(offset=0, startTime=0)
        annotation (Placement(transformation(extent={{-70,0},{-50,20}})));
      Modelica.Blocks.Noise.UniformNoise uniformNoise(
        samplePeriod=samplePeriod,
        y_min=0,
        y_max=1)
        annotation (Placement(transformation(extent={{12,-54},{32,-34}})));
      valveFailProto                  system_failure
        annotation (Placement(transformation(extent={{50,-8},{70,12}})));
      Modelica.Blocks.Logical.Greater greater1
        annotation (Placement(transformation(extent={{-28,30},{-8,10}})));
      Modelica.Blocks.Logical.Switch degradationModeswitch
        annotation (Placement(transformation(extent={{12,10},{32,30}})));
      Modelica.Blocks.Sources.CombiTimeTable NoHazardFunction(table=[3600,0; 2595600,
            0; 5187600,0; 7779600,0; 10371600,0; 12963600,0; 15555600,0; 18147600,0;
            20739600,0; 23331600,0; 25923600,0; 28515600,0; 31107600,0; 33699600,0;
            36291600,0; 38883600,0; 41475600,0; 44067600,0; 46659600,0; 49251600,0;
            51843600,0; 54435600,0; 57027600,0; 59619600,0; 62211600,0; 64803600,0;
            67395600,0; 69987600,0; 72579600,0; 75171600,0; 77763600,0; 80355600,0;
            82947600,0; 85539600,0; 88131600,0; 90723600,0; 93315600,0; 95907600,0;
            98499600,0; 101091600,0; 103683600,0; 106275600,0; 108867600,0; 111459600,
            0; 114051600,0; 116643600,0; 119235600,0; 121827600,0; 124419600,0; 127011600,
            0; 129603600,0; 132195600,0; 134787600,0; 137379600,0; 139971600,0; 142563600,
            0; 145155600,0; 147747600,0; 150339600,0; 152931600,0; 155523600,0; 158115600,
            0; 160707600,0; 163299600,0; 165891600,0; 168483600,0; 171075600,0; 173667600,
            0; 176259600,0; 178851600,0; 181443600,0; 184035600,0; 186627600,0; 189219600,
            0; 191811600,0; 194403600,0; 196995600,0; 199587600,0; 202179600,0; 204771600,
            0; 207363600,0; 209955600,0; 212547600,0; 215139600,0; 217731600,0; 220323600,
            0; 222915600,0; 225507600,0; 228099600,0; 230691600,0; 233283600,0; 235875600,
            0; 238467600,0; 241059600,0; 243651600,0; 246243600,0; 248835600,0; 251427600,
            0; 254019600,0; 256611600,0; 259203600,0; 261795600,0; 264387600,0; 266979600,
            0; 269571600,0; 272163600,0; 274755600,0; 277347600,0; 279939600,0; 282531600,
            0; 285123600,0; 287715600,0; 290307600,0; 292899600,0; 295491600,0; 298083600,
            0; 300675600,0; 303267600,0; 305859600,0; 308451600,0; 311043600,0; 313635600,
            0; 316227600,0])
                  annotation (Placement(transformation(extent={{-28,38},{-8,58}})));
      Modelica.Blocks.Sources.CombiTimeTable CumulHazardFunction_1(table=[3600,1.76124e-05,
            1.76124e-05; 2595600,0.000855354,0.000872967; 5187600,0.000443919,0.001316885;
            7779600,0.000358389,0.001675274; 10371600,0.000312179,0.001987453; 12963600,
            0.000281804,0.002269257; 15555600,0.000259763,0.00252902; 18147600,0.000242772,
            0.002771792; 20739600,0.000229126,0.003000918; 23331600,0.000217835,0.003218753;
            25923600,0.000208279,0.003427032; 28515600,0.000200046,0.003627078; 31107600,
            0.000192851,0.003819929; 33699600,0.000186489,0.004006418; 36291600,0.000180807,
            0.004187225; 38883600,0.00017569,0.004362915; 41475600,0.000171047,0.004533962;
            44067600,0.000166809,0.004700771; 46659600,0.000162919,0.00486369; 49251600,
            0.00015933,0.005023019; 51843600,0.000156004,0.005179023; 54435600,0.000152911,
            0.005331935; 57027600,0.000150024,0.005481958; 59619600,0.00014732,0.005629278;
            62211600,0.00014478,0.005774059; 64803600,0.000142389,0.005916447; 67395600,
            0.000140131,0.006056579; 69987600,0.000137995,0.006194574; 72579600,0.00013597,
            0.006330544; 75171600,0.000134046,0.00646459; 77763600,0.000132215,0.006596804;
            80355600,0.000130469,0.006727274; 82947600,0.000128803,0.006856077; 85539600,
            0.000127209,0.006983286; 88131600,0.000125683,0.007108969; 90723600,0.00012422,
            0.00723319; 93315600,0.000122816,0.007356006; 95907600,0.000121467,0.007477473;
            98499600,0.000120168,0.007597641; 101091600,0.000118918,0.007716559; 103683600,
            0.000117713,0.007834271; 106275600,0.00011655,0.007950821; 108867600,0.000115426,
            0.008066247; 111459600,0.000114341,0.008180588; 114051600,0.000113291,0.008293879;
            116643600,0.000112274,0.008406153; 119235600,0.000111289,0.008517442; 121827600,
            0.000110334,0.008627776; 124419600,0.000109408,0.008737184; 127011600,0.000108509,
            0.008845692; 129603600,0.000107635,0.008953328; 132195600,0.000106787,0.009060115;
            134787600,0.000105961,0.009166076; 137379600,0.000105158,0.009271234; 139971600,
            0.000104377,0.009375611; 142563600,0.000103615,0.009479226; 145155600,0.000102873,
            0.009582099; 147747600,0.00010215,0.009684249; 150339600,0.000101445,0.009785694;
            152931600,0.000100756,0.00988645; 155523600,0.000100084,0.009986534; 158115600,
            9.94283e-05,0.010085963; 160707600,9.87873e-05,0.01018475; 163299600,9.81609e-05,
            0.010282911; 165891600,9.75485e-05,0.010380459; 168483600,9.69495e-05,0.010477409;
            171075600,9.63636e-05,0.010573772; 173667600,9.57901e-05,0.010669562; 176259600,
            9.52287e-05,0.010764791; 178851600,9.46789e-05,0.01085947; 181443600,9.41403e-05,
            0.01095361; 184035600,9.36126e-05,0.011047223; 186627600,9.30953e-05,0.011140318;
            189219600,9.25882e-05,0.011232906; 191811600,9.20908e-05,0.011324997; 194403600,
            9.16029e-05,0.0114166; 196995600,9.11242e-05,0.011507724; 199587600,9.06544e-05,
            0.011598379; 202179600,9.01931e-05,0.011688572; 204771600,8.97402e-05,0.011778312;
            207363600,8.92954e-05,0.011867607; 209955600,8.88584e-05,0.011956466; 212547600,
            8.8429e-05,0.012044895; 215139600,8.8007e-05,0.012132902; 217731600,8.75922e-05,
            0.012220494; 220323600,8.71843e-05,0.012307678; 222915600,8.67832e-05,0.012394462;
            225507600,8.63886e-05,0.01248085; 228099600,8.60005e-05,0.012566851; 230691600,
            8.56186e-05,0.012652469; 233283600,8.52427e-05,0.012737712; 235875600,8.48727e-05,
            0.012822585; 238467600,8.45085e-05,0.012907093; 241059600,8.41498e-05,0.012991243;
            243651600,8.37966e-05,0.01307504; 246243600,8.34487e-05,0.013158488; 248835600,
            8.31059e-05,0.013241594; 251427600,8.27682e-05,0.013324363; 254019600,8.24354e-05,
            0.013406798; 256611600,8.21074e-05,0.013488905; 259203600,8.17841e-05,0.013570689;
            261795600,8.14653e-05,0.013652155; 264387600,8.1151e-05,0.013733306; 266979600,
            8.0841e-05,0.013814147; 269571600,8.05353e-05,0.013894682; 272163600,8.02337e-05,
            0.013974916; 274755600,7.99362e-05,0.014054852; 277347600,7.96426e-05,0.014134494;
            279939600,7.93529e-05,0.014213847; 282531600,7.9067e-05,0.014292914; 285123600,
            7.87848e-05,0.014371699; 287715600,7.85062e-05,0.014450205; 290307600,7.82312e-05,
            0.014528436; 292899600,7.79596e-05,0.014606396; 295491600,7.76914e-05,0.014684087;
            298083600,7.74266e-05,0.014761514; 300675600,7.7165e-05,0.014838679; 303267600,
            7.69065e-05,0.014915586; 305859600,7.66512e-05,0.014992237; 308451600,7.6399e-05,
            0.015068636; 311043600,7.61497e-05,0.015144785; 313635600,7.59034e-05,0.015220689;
            316227600,7.566e-05,0.015296349])
        annotation (Placement(transformation(extent={{-28,-24},{-8,-4}})));
      inner Modelica.Blocks.Noise.GlobalSeed globalSeed(fixedSeed=randomSeed)
        annotation (Placement(transformation(extent={{-98,74},{-78,94}})));
      Modelica.Blocks.Interfaces.BooleanOutput failreBooleanIndex
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    equation
      connect(uniformNoise.y, system_failure.random) annotation (Line(points={{33,-44},
              {43,-44},{43,-2.2},{51.2,-2.2}}, color={0,0,127}));
      connect(greater1.y, degradationModeswitch.u2)
        annotation (Line(points={{-7,20},{10,20}}, color={255,0,255}));
      connect(valvedelay6.y, greater1.u2) annotation (Line(points={{-49,40},{-38,40},
              {-38,28},{-30,28}}, color={0,0,127}));
      connect(clock4.y, greater1.u1) annotation (Line(points={{-49,10},{-38,10},{-38,
              20},{-30,20}}, color={0,0,127}));
      connect(NoHazardFunction.y[1], degradationModeswitch.u1)
        annotation (Line(points={{-7,48},{10,48},{10,28}}, color={0,0,127}));
      connect(CumulHazardFunction_1.y[1], degradationModeswitch.u3) annotation (
          Line(points={{-7,-14},{8,-14},{8,12},{10,12}}, color={0,0,127}));
      connect(degradationModeswitch.y, system_failure.hazard) annotation (Line(
            points={{33,20},{42,20},{42,6.6},{51.2,6.6}}, color={0,0,127}));
      connect(failreBooleanIndex, failreBooleanIndex)
        annotation (Line(points={{100,0},{100,0}}, color={255,0,255}));
      connect(system_failure.failreBooleanIndex, failreBooleanIndex)
        annotation (Line(points={{69,0},{100,0}}, color={255,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0}), Text(
              extent={{-60,42},{62,-46}},
              textColor={0,0,0},
              textString="%MOV
FailToOperate
Model")}),     Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MOV_failToOperate_Model;

    model systemDegradation_Model
      MOV_failToOperate_Model valveDegradation_Model1(
        randomSeed=dataValveDegradationModel.TCV_randomSeed_failToOperate,
        strategyChangeTime=dataValveDegradationModel.strategyChangeTime,
        CumulHazardFunction_1(table=[3600,1.76124e-05,1.76124e-05; 2595600,
              0.000855354,0.000872967; 5187600,0.000443919,0.001316885; 7779600,
              0.000358389,0.001675274; 10371600,0.000312179,0.001987453;
              12963600,0.000281804,0.002269257; 15555600,0.000259763,0.00252902;
              18147600,0.000242772,0.002771792; 20739600,0.000229126,
              0.003000918; 23331600,0.000217835,0.003218753; 25923600,
              0.000208279,0.003427032; 28515600,0.000200046,0.003627078;
              31107600,0.000192851,0.003819929; 33699600,0.000186489,
              0.004006418; 36291600,0.000180807,0.004187225; 38883600,
              0.00017569,0.004362915; 41475600,0.000171047,0.004533962;
              44067600,0.000166809,0.004700771; 46659600,0.000162919,0.00486369;
              49251600,0.00015933,0.005023019; 51843600,0.000156004,0.005179023;
              54435600,0.000152911,0.005331935; 57027600,0.000150024,
              0.005481958; 59619600,0.00014732,0.005629278; 62211600,0.00014478,
              0.005774059; 64803600,0.000142389,0.005916447; 67395600,
              0.000140131,0.006056579; 69987600,0.000137995,0.006194574;
              72579600,0.00013597,0.006330544; 75171600,0.000134046,0.00646459;
              77763600,0.000132215,0.006596804; 80355600,0.000130469,
              0.006727274; 82947600,0.000128803,0.006856077; 85539600,
              0.000127209,0.006983286; 88131600,0.000125683,0.007108969;
              90723600,0.00012422,0.00723319; 93315600,0.000122816,0.007356006;
              95907600,0.000121467,0.007477473; 98499600,0.000120168,
              0.007597641; 101091600,0.000118918,0.007716559; 103683600,
              0.000117713,0.007834271; 106275600,0.00011655,0.007950821;
              108867600,0.000115426,0.008066247; 111459600,0.000114341,
              0.008180588; 114051600,0.000113291,0.008293879; 116643600,
              0.000112274,0.008406153; 119235600,0.000111289,0.008517442;
              121827600,0.000110334,0.008627776; 124419600,0.500109408,
              0.008737184; 127011600,0.500108509,0.008845692; 129603600,
              0.500107635,0.008953328; 132195600,0.500106787,0.009060115;
              134787600,0.500105961,0.009166076; 137379600,0.500105158,
              0.009271234; 139971600,0.500104377,0.009375611; 142563600,
              0.500103615,0.009479226; 145155600,0.500102873,0.009582099;
              147747600,0.50010215,0.009684249; 150339600,0.500101445,
              0.009785694; 152931600,0.500100756,0.00988645; 155523600,
              0.500100084,0.009986534; 158115600,0.500099428,0.010085963;
              160707600,0.500098787,0.01018475; 163299600,0.500098161,
              0.010282911; 165891600,0.500097548,0.010380459; 168483600,
              0.50009695,0.010477409; 171075600,0.500096364,0.010573772;
              173667600,0.50009579,0.010669562; 176259600,0.500095229,
              0.010764791; 178851600,0.500094679,0.01085947; 181443600,
              0.50009414,0.01095361; 184035600,0.500093613,0.011047223;
              186627600,0.500093095,0.011140318; 189219600,0.500092588,
              0.011232906; 191811600,0.500092091,0.011324997; 194403600,
              0.500091603,0.0114166; 196995600,0.500091124,0.011507724;
              199587600,0.500090654,0.011598379; 202179600,0.500090193,
              0.011688572; 204771600,0.50008974,0.011778312; 207363600,
              0.500089295,0.011867607; 209955600,0.500088858,0.011956466;
              212547600,0.500088429,0.012044895; 215139600,0.500088007,
              0.012132902; 217731600,0.500087592,0.012220494; 220323600,
              0.500087184,0.012307678; 222915600,0.500086783,0.012394462;
              225507600,0.500086389,0.01248085; 228099600,0.500086001,
              0.012566851; 230691600,0.500085619,0.012652469; 233283600,
              0.500085243,0.012737712; 235875600,0.500084873,0.012822585;
              238467600,0.500084508,0.012907093; 241059600,0.50008415,
              0.012991243; 243651600,0.500083797,0.01307504; 246243600,
              0.500083449,0.013158488; 248835600,0.500083106,0.013241594;
              251427600,0.500082768,0.013324363; 254019600,0.500082435,
              0.013406798; 256611600,0.500082107,0.013488905; 259203600,
              0.500081784,0.013570689; 261795600,0.500081465,0.013652155;
              264387600,0.500081151,0.013733306; 266979600,0.500080841,
              0.013814147; 269571600,0.500080535,0.013894682; 272163600,
              0.500080234,0.013974916; 274755600,0.500079936,0.014054852;
              277347600,0.500079643,0.014134494; 279939600,0.500079353,
              0.014213847; 282531600,0.500079067,0.014292914; 285123600,
              0.500078785,0.014371699; 287715600,0.500078506,0.014450205;
              290307600,0.500078231,0.014528436; 292899600,0.50007796,
              0.014606396; 295491600,0.500077691,0.014684087; 298083600,
              0.500077427,0.014761514; 300675600,0.500077165,0.014838679;
              303267600,0.500076907,0.014915586; 305859600,0.500076651,
              0.014992237; 308451600,0.500076399,0.015068636; 311043600,
              0.50007615,0.015144785; 313635600,0.500075903,0.015220689;
              316227600,0.50007566,0.015296349]))
        annotation (Placement(transformation(extent={{-94,66},{-74,86}})));
      MOV_failToOperate_Model valveDegradation_Model2(
        randomSeed=dataValveDegradationModel.LPTBV1_randomSeed_failToOperate,
        strategyChangeTime=dataValveDegradationModel.strategyChangeTime,
        CumulHazardFunction_1(table=[3600,1.76124e-05,1.76124e-05; 2595600,
              0.000855354,0.000872967; 5187600,0.000443919,0.001316885; 7779600,
              0.000358389,0.001675274; 10371600,0.000312179,0.001987453;
              12963600,0.000281804,0.002269257; 15555600,0.000259763,0.00252902;
              18147600,0.000242772,0.002771792; 20739600,0.000229126,
              0.003000918; 23331600,0.000217835,0.003218753; 25923600,
              0.000208279,0.003427032; 28515600,0.000200046,0.003627078;
              31107600,0.000192851,0.003819929; 33699600,0.000186489,
              0.004006418; 36291600,0.000180807,0.004187225; 38883600,
              0.00017569,0.004362915; 41475600,0.000171047,0.004533962;
              44067600,0.000166809,0.004700771; 46659600,0.000162919,0.00486369;
              49251600,0.00015933,0.005023019; 51843600,0.000156004,0.005179023;
              54435600,0.000152911,0.005331935; 57027600,0.000150024,
              0.005481958; 59619600,0.00014732,0.005629278; 62211600,0.00014478,
              0.005774059; 64803600,0.000142389,0.005916447; 67395600,
              0.000140131,0.006056579; 69987600,0.000137995,0.006194574;
              72579600,0.00013597,0.006330544; 75171600,0.000134046,0.00646459;
              77763600,0.000132215,0.006596804; 80355600,0.000130469,
              0.006727274; 82947600,0.000128803,0.006856077; 85539600,
              0.000127209,0.006983286; 88131600,0.000125683,0.007108969;
              90723600,0.00012422,0.00723319; 93315600,0.000122816,0.007356006;
              95907600,0.000121467,0.007477473; 98499600,0.000120168,
              0.007597641; 101091600,0.000118918,0.007716559; 103683600,
              0.000117713,0.007834271; 106275600,0.00011655,0.007950821;
              108867600,0.000115426,0.008066247; 111459600,0.000114341,
              0.008180588; 114051600,0.000113291,0.008293879; 116643600,
              0.000112274,0.008406153; 119235600,0.000111289,0.008517442;
              121827600,0.000110334,0.008627776; 124419600,0.000109408,
              0.008737184; 127011600,0.000108509,0.008845692; 129603600,
              0.000107635,0.008953328; 132195600,0.000106787,0.009060115;
              134787600,0.000105961,0.009166076; 137379600,0.000105158,
              0.009271234; 139971600,0.000104377,0.009375611; 142563600,
              0.000103615,0.009479226; 145155600,0.000102873,0.009582099;
              147747600,0.00010215,0.009684249; 150339600,0.000101445,
              0.009785694; 152931600,0.000100756,0.00988645; 155523600,
              0.000100084,0.009986534; 158115600,9.94283e-05,0.010085963;
              160707600,9.87873e-05,0.01018475; 163299600,9.81609e-05,
              0.010282911; 165891600,9.75485e-05,0.010380459; 168483600,
              9.69495e-05,0.010477409; 171075600,9.63636e-05,0.010573772;
              173667600,9.57901e-05,0.010669562; 176259600,9.52287e-05,
              0.010764791; 178851600,9.46789e-05,0.01085947; 181443600,
              9.41403e-05,0.01095361; 184035600,9.36126e-05,0.011047223;
              186627600,9.30953e-05,0.011140318; 189219600,9.25882e-05,
              0.011232906; 191811600,9.20908e-05,0.011324997; 194403600,
              9.16029e-05,0.0114166; 196995600,9.11242e-05,0.011507724;
              199587600,9.06544e-05,0.011598379; 202179600,0.500090193,
              0.011688572; 204771600,0.50008974,0.011778312; 207363600,
              0.500089295,0.011867607; 209955600,0.500088858,0.011956466;
              212547600,0.500088429,0.012044895; 215139600,0.500088007,
              0.012132902; 217731600,0.500087592,0.012220494; 220323600,
              0.500087184,0.012307678; 222915600,0.500086783,0.012394462;
              225507600,0.500086389,0.01248085; 228099600,0.500086001,
              0.012566851; 230691600,0.500085619,0.012652469; 233283600,
              0.500085243,0.012737712; 235875600,0.500084873,0.012822585;
              238467600,0.500084508,0.012907093; 241059600,0.50008415,
              0.012991243; 243651600,0.500083797,0.01307504; 246243600,
              0.500083449,0.013158488; 248835600,0.500083106,0.013241594;
              251427600,0.500082768,0.013324363; 254019600,0.500082435,
              0.013406798; 256611600,0.500082107,0.013488905; 259203600,
              0.500081784,0.013570689; 261795600,0.500081465,0.013652155;
              264387600,0.500081151,0.013733306; 266979600,0.500080841,
              0.013814147; 269571600,0.500080535,0.013894682; 272163600,
              0.500080234,0.013974916; 274755600,0.500079936,0.014054852;
              277347600,0.500079643,0.014134494; 279939600,0.500079353,
              0.014213847; 282531600,0.500079067,0.014292914; 285123600,
              0.500078785,0.014371699; 287715600,0.500078506,0.014450205;
              290307600,0.500078231,0.014528436; 292899600,0.50007796,
              0.014606396; 295491600,0.500077691,0.014684087; 298083600,
              0.500077427,0.014761514; 300675600,0.500077165,0.014838679;
              303267600,0.500076907,0.014915586; 305859600,0.500076651,
              0.014992237; 308451600,0.500076399,0.015068636; 311043600,
              0.50007615,0.015144785; 313635600,0.500075903,0.015220689;
              316227600,0.50007566,0.015296349]))
        annotation (Placement(transformation(extent={{-94,40},{-74,60}})));
      MOV_failToOperate_Model valveDegradation_Model3(
        randomSeed=dataValveDegradationModel.LPTBV2_randomSeed_failToOperate,
        strategyChangeTime=dataValveDegradationModel.strategyChangeTime,
        CumulHazardFunction_1(table=[3600,1.76124e-05,1.76124e-05; 2595600,
              0.000855354,0.000872967; 5187600,0.000443919,0.001316885; 7779600,
              0.000358389,0.001675274; 10371600,0.000312179,0.001987453;
              12963600,0.000281804,0.002269257; 15555600,0.000259763,0.00252902;
              18147600,0.000242772,0.002771792; 20739600,0.000229126,
              0.003000918; 23331600,0.000217835,0.003218753; 25923600,
              0.000208279,0.003427032; 28515600,0.000200046,0.003627078;
              31107600,0.000192851,0.003819929; 33699600,0.000186489,
              0.004006418; 36291600,0.000180807,0.004187225; 38883600,
              0.00017569,0.004362915; 41475600,0.000171047,0.004533962;
              44067600,0.000166809,0.004700771; 46659600,0.000162919,0.00486369;
              49251600,0.00015933,0.005023019; 51843600,0.000156004,0.005179023;
              54435600,0.000152911,0.005331935; 57027600,0.000150024,
              0.005481958; 59619600,0.00014732,0.005629278; 62211600,0.00014478,
              0.005774059; 64803600,0.000142389,0.005916447; 67395600,
              0.000140131,0.006056579; 69987600,0.000137995,0.006194574;
              72579600,0.00013597,0.006330544; 75171600,0.000134046,0.00646459;
              77763600,0.000132215,0.006596804; 80355600,0.000130469,
              0.006727274; 82947600,0.000128803,0.006856077; 85539600,
              0.000127209,0.006983286; 88131600,0.000125683,0.007108969;
              90723600,0.00012422,0.00723319; 93315600,0.000122816,0.007356006;
              95907600,0.000121467,0.007477473; 98499600,0.000120168,
              0.007597641; 101091600,0.000118918,0.007716559; 103683600,
              0.000117713,0.007834271; 106275600,0.00011655,0.007950821;
              108867600,0.000115426,0.008066247; 111459600,0.000114341,
              0.008180588; 114051600,0.000113291,0.008293879; 116643600,
              0.000112274,0.008406153; 119235600,0.000111289,0.008517442;
              121827600,0.000110334,0.008627776; 124419600,0.000109408,
              0.008737184; 127011600,0.000108509,0.008845692; 129603600,
              0.000107635,0.008953328; 132195600,0.000106787,0.009060115;
              134787600,0.000105961,0.009166076; 137379600,0.000105158,
              0.009271234; 139971600,0.000104377,0.009375611; 142563600,
              0.000103615,0.009479226; 145155600,0.000102873,0.009582099;
              147747600,0.00010215,0.009684249; 150339600,0.000101445,
              0.009785694; 152931600,0.000100756,0.00988645; 155523600,
              0.000100084,0.009986534; 158115600,9.94283e-05,0.010085963;
              160707600,9.87873e-05,0.01018475; 163299600,9.81609e-05,
              0.010282911; 165891600,9.75485e-05,0.010380459; 168483600,
              9.69495e-05,0.010477409; 171075600,9.63636e-05,0.010573772;
              173667600,9.57901e-05,0.010669562; 176259600,9.52287e-05,
              0.010764791; 178851600,9.46789e-05,0.01085947; 181443600,
              9.41403e-05,0.01095361; 184035600,9.36126e-05,0.011047223;
              186627600,9.30953e-05,0.011140318; 189219600,9.25882e-05,
              0.011232906; 191811600,9.20908e-05,0.011324997; 194403600,
              9.16029e-05,0.0114166; 196995600,9.11242e-05,0.011507724;
              199587600,9.06544e-05,0.011598379; 202179600,9.01931e-05,
              0.011688572; 204771600,8.97402e-05,0.011778312; 207363600,
              8.92954e-05,0.011867607; 209955600,8.88584e-05,0.011956466;
              212547600,8.8429e-05,0.012044895; 215139600,8.8007e-05,
              0.012132902; 217731600,8.75922e-05,0.012220494; 220323600,
              8.71843e-05,0.012307678; 222915600,8.67832e-05,0.012394462;
              225507600,8.63886e-05,0.01248085; 228099600,8.60005e-05,
              0.012566851; 230691600,8.56186e-05,0.012652469; 233283600,
              8.52427e-05,0.012737712; 235875600,8.48727e-05,0.012822585;
              238467600,8.45085e-05,0.012907093; 241059600,8.41498e-05,
              0.012991243; 243651600,8.37966e-05,0.01307504; 246243600,
              8.34487e-05,0.013158488; 248835600,8.31059e-05,0.013241594;
              251427600,8.27682e-05,0.013324363; 254019600,0.500082435,
              0.013406798; 256611600,0.500082107,0.013488905; 259203600,
              0.500081784,0.013570689; 261795600,0.500081465,0.013652155;
              264387600,0.500081151,0.013733306; 266979600,0.500080841,
              0.013814147; 269571600,0.500080535,0.013894682; 272163600,
              0.500080234,0.013974916; 274755600,0.500079936,0.014054852;
              277347600,0.500079643,0.014134494; 279939600,0.500079353,
              0.014213847; 282531600,0.500079067,0.014292914; 285123600,
              0.500078785,0.014371699; 287715600,0.500078506,0.014450205;
              290307600,0.500078231,0.014528436; 292899600,0.50007796,
              0.014606396; 295491600,0.500077691,0.014684087; 298083600,
              0.500077427,0.014761514; 300675600,0.500077165,0.014838679;
              303267600,0.500076907,0.014915586; 305859600,0.500076651,
              0.014992237; 308451600,0.500076399,0.015068636; 311043600,
              0.50007615,0.015144785; 313635600,0.500075903,0.015220689;
              316227600,0.50007566,0.015296349]))
        annotation (Placement(transformation(extent={{-94,14},{-74,34}})));
      Data.dataValveDegradationModel dataValveDegradationModel
        annotation (Placement(transformation(extent={{70,76},{90,96}})));
      Modelica.Blocks.Logical.Or or1
        annotation (Placement(transformation(extent={{-42,58},{-22,78}})));
      Modelica.Blocks.Logical.Or or2
        annotation (Placement(transformation(extent={{6,34},{26,54}})));
      Modelica.Blocks.Interfaces.BooleanOutput y
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      MOV_failToRemainOpen_Model mOV_failToRemainOpen_Model(randomSeed=
            dataValveDegradationModel.TCV_randomSeed_failToRemainOpen)
        annotation (Placement(transformation(extent={{-94,-28},{-74,-8}})));
      MOV_failToRemainOpen_Model mOV_failToRemainOpen_Model1(randomSeed=
            dataValveDegradationModel.LPTBV1_randomSeed_failToRemainOpen)
        annotation (Placement(transformation(extent={{-94,-56},{-74,-36}})));
      MOV_failToRemainOpen_Model mOV_failToRemainOpen_Model2(randomSeed=
            dataValveDegradationModel.LPTBV2_randomSeed_failToRemainOpen)
        annotation (Placement(transformation(extent={{-94,-84},{-74,-64}})));
      Modelica.Blocks.Logical.Or or3
        annotation (Placement(transformation(extent={{-42,-36},{-22,-16}})));
      Modelica.Blocks.Logical.Or or4
        annotation (Placement(transformation(extent={{6,-60},{26,-40}})));
      Modelica.Blocks.Logical.Or or5
        annotation (Placement(transformation(extent={{48,-10},{68,10}})));
    equation
      connect(valveDegradation_Model1.failreBooleanIndex, or1.u1) annotation (
          Line(points={{-74,76},{-50,76},{-50,68},{-44,68}}, color={255,0,255}));
      connect(valveDegradation_Model2.failreBooleanIndex, or1.u2) annotation (
          Line(points={{-74,50},{-50,50},{-50,60},{-44,60}}, color={255,0,255}));
      connect(or2.u1, or1.y) annotation (Line(points={{4,44},{-14,44},{-14,68},
              {-21,68}}, color={255,0,255}));
      connect(or2.u2, valveDegradation_Model3.failreBooleanIndex) annotation (
          Line(points={{4,36},{-14,36},{-14,24},{-74,24}}, color={255,0,255}));
      connect(or5.y, y)
        annotation (Line(points={{69,0},{100,0}}, color={255,0,255}));
      connect(or2.y, or5.u1) annotation (Line(points={{27,44},{40,44},{40,0},{
              46,0}}, color={255,0,255}));
      connect(or4.y, or5.u2) annotation (Line(points={{27,-50},{40,-50},{40,-8},
              {46,-8}}, color={255,0,255}));
      connect(or3.y, or4.u1) annotation (Line(points={{-21,-26},{-2,-26},{-2,
              -50},{4,-50}}, color={255,0,255}));
      connect(mOV_failToRemainOpen_Model2.failToRemainOpenIndex, or4.u2)
        annotation (Line(points={{-74,-74},{-2,-74},{-2,-58},{4,-58}}, color={
              255,0,255}));
      connect(mOV_failToRemainOpen_Model1.failToRemainOpenIndex, or3.u2)
        annotation (Line(points={{-74,-46},{-50,-46},{-50,-34},{-44,-34}},
            color={255,0,255}));
      connect(mOV_failToRemainOpen_Model.failToRemainOpenIndex, or3.u1)
        annotation (Line(points={{-74,-18},{-52,-18},{-52,-26},{-44,-26}},
            color={255,0,255}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
                extent={{-100,100},{100,-100}}, lineColor={0,0,0})}),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=316227600, __Dymola_Algorithm="Dassl"));
    end systemDegradation_Model;

    model MOV_failToRemainOpen_Model "Separate_System_Failure_Modeling"
      parameter Integer randomSeed = 1234 "random Seed";
      parameter SI.Time samplePeriod = 2592000 "2592000 sec = 720 hours";
      Modelica.Blocks.Noise.UniformNoise uniformNoise(
        samplePeriod=samplePeriod,
        y_min=0,
        y_max=1)
        annotation (Placement(transformation(extent={{-28,-30},{-8,-10}})));
      valveFailProto                  system_failure
        annotation (Placement(transformation(extent={{18,-12},{38,8}})));
      Modelica.Blocks.Sources.CombiTimeTable NoHazardFunction(table=[3600,0; 2595600,
            0; 5187600,0; 7779600,0; 10371600,0; 12963600,0; 15555600,0; 18147600,0;
            20739600,0; 23331600,0; 25923600,0; 28515600,0; 31107600,0; 33699600,0;
            36291600,0; 38883600,0; 41475600,0; 44067600,0; 46659600,0; 49251600,0;
            51843600,0; 54435600,0; 57027600,0; 59619600,0; 62211600,0; 64803600,0;
            67395600,0; 69987600,0; 72579600,0; 75171600,0; 77763600,0; 80355600,0;
            82947600,0; 85539600,0; 88131600,0; 90723600,0; 93315600,0; 95907600,0;
            98499600,0; 101091600,0; 103683600,0; 106275600,0; 108867600,0; 111459600,
            0; 114051600,0; 116643600,0; 119235600,0; 121827600,0; 124419600,0; 127011600,
            0; 129603600,0; 132195600,0; 134787600,0; 137379600,0; 139971600,0; 142563600,
            0; 145155600,0; 147747600,0; 150339600,0; 152931600,0; 155523600,0; 158115600,
            0; 160707600,0; 163299600,0; 165891600,0; 168483600,0; 171075600,0; 173667600,
            0; 176259600,0; 178851600,0; 181443600,0; 184035600,0; 186627600,0; 189219600,
            0; 191811600,0; 194403600,0; 196995600,0; 199587600,0; 202179600,0; 204771600,
            0; 207363600,0; 209955600,0; 212547600,0; 215139600,0; 217731600,0; 220323600,
            0; 222915600,0; 225507600,0; 228099600,0; 230691600,0; 233283600,0; 235875600,
            0; 238467600,0; 241059600,0; 243651600,0; 246243600,0; 248835600,0; 251427600,
            0; 254019600,0; 256611600,0; 259203600,0; 261795600,0; 264387600,0; 266979600,
            0; 269571600,0; 272163600,0; 274755600,0; 277347600,0; 279939600,0; 282531600,
            0; 285123600,0; 287715600,0; 290307600,0; 292899600,0; 295491600,0; 298083600,
            0; 300675600,0; 303267600,0; 305859600,0; 308451600,0; 311043600,0; 313635600,
            0; 316227600,0])
                  annotation (Placement(transformation(extent={{-28,10},{-8,30}})));
      inner Modelica.Blocks.Noise.GlobalSeed globalSeed(fixedSeed=randomSeed)
        annotation (Placement(transformation(extent={{-98,74},{-78,94}})));
      Modelica.Blocks.Interfaces.BooleanOutput failToRemainOpenIndex
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    equation
      connect(uniformNoise.y, system_failure.random) annotation (Line(points={{-7,-20},
              {14,-20},{14,-6.2},{19.2,-6.2}}, color={0,0,127}));
      connect(failToRemainOpenIndex, failToRemainOpenIndex)
        annotation (Line(points={{100,0},{100,0}}, color={255,0,255}));
      connect(system_failure.failreBooleanIndex, failToRemainOpenIndex) annotation (
         Line(points={{37,-4},{68,-4},{68,0},{100,0}}, color={255,0,255}));
      connect(NoHazardFunction.y[1], system_failure.hazard) annotation (Line(points=
             {{-7,20},{14,20},{14,2.6},{19.2,2.6}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0}), Text(
              extent={{-60,42},{62,-46}},
              textColor={0,0,0},
              textString="%MOV
FailToRemain
Open
Model")}),     Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MOV_failToRemainOpen_Model;
  end Component_Degradation;

  model
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type2
    "Pressure Control done!!!"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable
        ControlSystems.CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TCVcontrol_CompDegradation_type1
        CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{64,122},{84,142}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={238,66})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{244,-62},{224,-42}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      redeclare model EfficiencyChar =
          TRANSFORM.Fluid.Machines.BaseClasses.PumpCharacteristics.Efficiency.Degradation,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
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
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
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
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=30)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-108,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-19})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
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
      V=5,
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
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15)                                  annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{90,122},{110,142}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-104,-42},{-84,-22}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={44,-32})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{258,100},{258,-98},{-34,-98},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-20,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-30,76},{-80,76},{-80,112},{-85,112}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(
        points={{-10,-58},{-24,-58}},
        color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-30,-44},{-56,-44},{-56,-74},{-108,-74},{-108,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{234,-62},
            {234,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-26},{242,-42},{241,-42}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
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
    connect(port_a, volume.port_a)
      annotation (Line(points={{-140,48},{-68,48}}, color={0,127,255}));
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
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-12}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{248,100},{248,16},{186,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-58},{2,-58}},                     color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={{94,-26},
            {94,-32},{54,-32}},          color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{34,-32},{-84,-32}},color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-30,-42},{44,-42},{44,-35.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(actuatorBus.openingLPTv, LPT1_Bypass.opening) annotation (Line(
        points={{30,100},{110,100},{110,-16},{102,-16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(generator.shaft, LPT2.shaft_b) annotation (Line(points={{238.1,55.9},
            {238.1,34},{218,34}}, color={0,0,0}));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-98,-58},{-44,-58}}, color={0,127,255}));
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
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type2;

  model
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type3
    "type2 + STisentropic efficiency is changing due to degradation"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable
        ControlSystems.CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TCVcontrol_CompDegradation_type1
        CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{64,122},{84,142}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_HPT
          (start_coef=componentDegradation.HPT_start_coeff),
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={238,66})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{244,-62},{224,-42}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      redeclare model EfficiencyChar =
          TRANSFORM.Fluid.Machines.BaseClasses.PumpCharacteristics.Efficiency.Degradation,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
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
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));
    Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
      annotation (Placement(transformation(extent={{-106,108},{-86,116}})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT1(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_LPT1
          (start_coef=componentDegradation.LPT1_start_eff),
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15) annotation (Placement(transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=30)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-108,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-19})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
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
      V=5,
      p_start=1500000,
      T_start=423.15)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={178,50})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT2(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_LPT2
          (start_coef=componentDegradation.LPT2_start_eff),
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{90,122},{110,142}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-104,-42},{-84,-22}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={44,-32})));
    Data.ComponentDegradation componentDegradation
      annotation (Placement(transformation(extent={{120,122},{140,142}})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{258,100},{258,-98},{-34,-98},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-20,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-30,76},{-80,76},{-80,112},{-85,112}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(
        points={{-10,-58},{-24,-58}},
        color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-30,-44},{-56,-44},{-56,-74},{-108,-74},{-108,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{234,-62},
            {234,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-26},{242,-42},{241,-42}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
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
    connect(port_a, volume.port_a)
      annotation (Line(points={{-140,48},{-68,48}}, color={0,127,255}));
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
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-12}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{248,100},{248,16},{186,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-58},{2,-58}},                     color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={{94,-26},
            {94,-32},{54,-32}},          color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{34,-32},{-84,-32}},color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-30,-42},{44,-42},{44,-35.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(actuatorBus.openingLPTv, LPT1_Bypass.opening) annotation (Line(
        points={{30,100},{110,100},{110,-16},{102,-16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(generator.shaft, LPT2.shaft_b) annotation (Line(points={{238.1,55.9},
            {238.1,34},{218,34}}, color={0,0,0}));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-98,-58},{-44,-58}}, color={0,127,255}));
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
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type3;

  model
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type4
    "type3 + Valve (MOV) setpoint drift modeling"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable
        ControlSystems.CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TCVcontrol_CompDegradation_type1
        CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{64,122},{84,142}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_HPT
          (
          lambda=componentDegradation.HPT_lambda,
          lambda_coef=componentDegradation.HPT_coef,
          start_coef=componentDegradation.HPT_start_coeff,
          oper_time=componentDegradation.Strategy_Change_Time),
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={238,66})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{244,-62},{224,-42}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      redeclare model EfficiencyChar =
          TRANSFORM.Fluid.Machines.BaseClasses.PumpCharacteristics.Efficiency.Constant,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
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

    TRANSFORM.Fluid.Valves.ValveLinear_Aging TCV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50,
      r1=componentDegradation.LPTBV1_random_coeff) annotation (Placement(
          transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));
    Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
      annotation (Placement(transformation(extent={{-106,108},{-86,116}})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT1(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_LPT1
          (
          lambda=componentDegradation.LPT1_lambda,
          lambda_coef=componentDegradation.LPT1_coef,
          start_coef=componentDegradation.LPT1_start_eff),
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15) annotation (Placement(transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear_Aging
                                       LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=30,
      r1=componentDegradation.LPTBV1_random_coeff)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-108,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-19})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
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
      V=5,
      p_start=1500000,
      T_start=423.15)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={178,50})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT2(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_LPT2
          (
          lambda=componentDegradation.LPT2_lambda,
          lambda_coef=componentDegradation.LPT2_coef,
          start_coef=componentDegradation.LPT2_start_eff),
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear_Aging
                                       LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5,
      r1=componentDegradation.LPTBV2_random_coeff)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{90,122},{110,142}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-104,-42},{-84,-22}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={44,-32})));
    Data.ComponentDegradation componentDegradation
      annotation (Placement(transformation(extent={{120,122},{140,142}})));
    inner Modelica.Blocks.Noise.GlobalSeed globalSeed
      annotation (Placement(transformation(extent={{-140,116},{-120,136}})));
    Modelica.Blocks.Noise.UniformNoise uniformNoise(
      samplePeriod=1000,
      y_min=0,
      y_max=1)
      annotation (Placement(transformation(extent={{154,112},{174,132}})));
    Modelica.Blocks.Sources.CombiTimeTable CumulHazardFunction(table=[1,
          1.76124e-05,1.76124e-05; 1001,0.001043086,0.001060698; 2001,
          0.000539738,0.001600437; 3001,0.000435809,0.002036246; 4001,
          0.00037967,0.002415916; 5001,0.000342772,0.002758688; 6001,0.000316,
          0.003074689; 7001,0.000295364,0.003370053; 8001,0.000278791,
          0.003648844; 9001,0.000265079,0.003913923; 10001,0.000253474,
          0.004167398; 11001,0.000243477,0.004410875; 12001,0.000234741,
          0.004645616; 13001,0.000227016,0.004872633; 14001,0.000220118,
          0.00509275; 15001,0.000213905,0.005306655; 16001,0.000208269,
          0.005514924; 17001,0.000203123,0.005718047; 18001,0.0001984,
          0.005916447; 19001,0.000194044,0.006110491; 20001,0.000190007,
          0.006300498; 21001,0.000186252,0.00648675; 22001,0.000182748,
          0.006669498; 23001,0.000179466,0.006848963; 24001,0.000176383,
          0.007025347; 25001,0.000173481,0.007198827; 26001,0.000170741,
          0.007369568; 27001,0.000168148,0.007537716; 28001,0.00016569,
          0.007703406; 29001,0.000163355,0.007866761; 30001,0.000161133,
          0.008027894; 31001,0.000159015,0.008186909; 32001,0.000156992,
          0.008343902; 33001,0.000155059,0.00849896; 34001,0.000153207,
          0.008652168; 35001,0.000151432,0.0088036; 36001,0.000149728,
          0.008953328; 37001,0.000148091,0.009101419; 38001,0.000146515,
          0.009247934; 39001,0.000144998,0.009392932; 40001,0.000143536,
          0.009536468; 41001,0.000142125,0.009678593; 42001,0.000140762,
          0.009819355; 43001,0.000139445,0.0099588; 44001,0.000138171,
          0.01009697; 45001,0.000136937,0.010233908; 46001,0.000135743,
          0.01036965; 47001,0.000134584,0.010504235; 48001,0.000133461,
          0.010637695; 49001,0.00013237,0.010770065; 50001,0.000131311,
          0.010901376; 51001,0.000130281,0.011031657; 52001,0.00012928,
          0.011160937; 53001,0.000128306,0.011289243; 54001,0.000127358,
          0.0114166; 55001,0.000126434,0.011543034; 56001,0.000125534,
          0.011668568; 57001,0.000124657,0.011793225; 58001,0.000123801,
          0.011917027; 59001,0.000122967,0.012039993; 60001,0.000122152,
          0.012162145; 61001,0.000121356,0.012283501; 62001,0.000120579,
          0.01240408; 63001,0.000119819,0.012523899; 64001,0.000119076,
          0.012642975; 65001,0.00011835,0.012761325; 66001,0.000117639,
          0.012878964; 67001,0.000116944,0.012995908; 68001,0.000116263,
          0.013112171; 69001,0.000115596,0.013227767; 70001,0.000114943,
          0.01334271; 71001,0.000114303,0.013457013; 72001,0.000113676,
          0.013570689; 73001,0.000113061,0.013683751; 74001,0.000112458,
          0.013796209; 75001,0.000111866,0.013908075; 76001,0.000111286,
          0.014019361; 77001,0.000110716,0.014130077; 78001,0.000110157,
          0.014240235; 79001,0.000109608,0.014349842; 80001,0.000109069,
          0.014458911; 81001,0.000108539,0.01456745; 82001,0.000108018,
          0.014675468; 83001,0.000107507,0.014782975; 84001,0.000107004,
          0.014889979; 85001,0.000106509,0.014996488; 86001,0.000106023,
          0.015102511])
      annotation (Placement(transformation(extent={{154,144},{174,164}})));
    Component_Degradation.valveFail system_failure
      annotation (Placement(transformation(extent={{192,128},{212,148}})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{258,100},{258,-98},{-34,-98},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-20,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-30,76},{-80,76},{-80,112},{-85,112}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(
        points={{-10,-58},{-24,-58}},
        color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-30,-44},{-56,-44},{-56,-74},{-108,-74},{-108,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{234,-62},
            {234,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-26},{242,-42},{241,-42}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
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
    connect(port_a, volume.port_a)
      annotation (Line(points={{-140,48},{-68,48}}, color={0,127,255}));
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
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-12}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(actuatorBus.Divert_Valve_Position, LPT2_Bypass.opening) annotation (
        Line(
        points={{30,100},{248,100},{248,16},{186,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-58},{2,-58}},                     color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={{94,-26},
            {94,-32},{54,-32}},          color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{34,-32},{-84,-32}},color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-30,-42},{44,-42},{44,-35.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(actuatorBus.openingLPTv, LPT1_Bypass.opening) annotation (Line(
        points={{30,100},{110,100},{110,-16},{102,-16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(generator.shaft, LPT2.shaft_b) annotation (Line(points={{238.1,55.9},
            {238.1,34},{218,34}}, color={0,0,0}));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-98,-58},{-44,-58}}, color={0,127,255}));
    connect(CumulHazardFunction.y[1], system_failure.hazard) annotation (Line(
          points={{175,154},{185,154},{185,142.6},{193.2,142.6}}, color={0,0,
            127}));
    connect(uniformNoise.y, system_failure.random) annotation (Line(points={{
            175,122},{185,122},{185,133.8},{193.2,133.8}}, color={0,0,127}));
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
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type4;

  model
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type5_test
    "type3 + Valve (MOV) setpoint drift modeling_based on hazard function"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable
        ControlSystems.CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TCVcontrol_CompDegradation_type1
        CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);
    Real tcv_derv;
    Real ltv1_derv;
    Real ltv2_derv;

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{64,122},{84,142}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_HPT,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={238,66})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{244,-62},{224,-42}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      redeclare model EfficiencyChar =
          TRANSFORM.Fluid.Machines.BaseClasses.PumpCharacteristics.Efficiency.Constant,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
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
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));
    Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
      annotation (Placement(transformation(extent={{-122,100},{-86,116}})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT1(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_LPT1,
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15) annotation (Placement(transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=30)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-108,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-19})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
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
      V=5,
      p_start=1500000,
      T_start=423.15)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={178,50})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT2(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_LPT2,
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{90,122},{110,142}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-104,-42},{-84,-22}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={44,-32})));
    Data.ComponentDegradation componentDegradation(
      HPT_lambda=0.00000001,
      LPT1_lambda=0.00000001,
      LPT2_lambda=0.00000001)
      annotation (Placement(transformation(extent={{120,122},{140,142}})));
    Component_Degradation.valve_degradation valve_degradation annotation (
        Placement(transformation(
          extent={{6,-6},{-6,6}},
          rotation=90,
          origin={-4,60})));
    Component_Degradation.valve_degradation valve_degradation1 annotation (
        Placement(transformation(
          extent={{6,-6},{-6,6}},
          rotation=90,
          origin={110,2})));
    Component_Degradation.valve_degradation valve_degradation2 annotation (
        Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=180,
          origin={206,16})));
    Component_Degradation.systemFail_Index systemFail_Index
      annotation (Placement(transformation(extent={{200,110},{248,160}})));
    Modelica.Blocks.Sources.RealExpression tcv_derv_send(y=tcv_derv)
      annotation (Placement(transformation(extent={{166,138},{190,152}})));
    Modelica.Blocks.Sources.RealExpression ltv1_derv_send(y=ltv1_derv)
      annotation (Placement(transformation(extent={{166,126},{190,140}})));
    Modelica.Blocks.Sources.RealExpression ltv2_derv_send(y=ltv2_derv)
      annotation (Placement(transformation(extent={{166,114},{190,128}})));
  initial equation

  equation
    port_e.W = generator.power;
    if time >= 1000 then
      tcv_derv = der(TCV.opening);
      ltv1_derv = der(LPT1_Bypass.opening);
      ltv2_derv = der(LPT2_Bypass.opening);
    else
      tcv_derv = 1;
      ltv1_derv = 1;
      ltv2_derv = 1;
    end if;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{258,100},{258,-98},{-34,-98},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-20,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-30,76},{-62,76},{-62,108},{-84.2,108}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(
        points={{-10,-58},{-24,-58}},
        color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-30,-44},{-56,-44},{-56,-74},{-108,-74},{-108,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{234,-62},
            {234,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-26},{242,-42},{241,-42}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,48},{-46,
            48},{-46,72},{-66,72}},
                                color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,48},{-34,48},{-34,40},{-12,40}},
                                                   color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,48},{
            -34,48},{-34,62},{-14,62},{-14,66}},
                                         color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-140,48},{-68,48}}, color={0,127,255}));
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
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-12}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-58},{2,-58}},                     color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={{94,-26},
            {94,-32},{54,-32}},          color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{34,-32},{-84,-32}},color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-30,-42},{44,-42},{44,-35.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(generator.shaft, LPT2.shaft_b) annotation (Line(points={{238.1,55.9},
            {238.1,34},{218,34}}, color={0,0,0}));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-98,-58},{-44,-58}}, color={0,127,255}));
    connect(TCV.opening, valve_degradation.open_out)
      annotation (Line(points={{-4,46.4},{-4,54.6}}, color={0,0,127}));
    connect(actuatorBus.opening_TCV, valve_degradation.open_in) annotation (
        Line(
        points={{30.1,100.1},{-4,100.1},{-4,65.28}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
    connect(valve_degradation1.open_out, LPT1_Bypass.opening) annotation (Line(
          points={{110,-3.4},{110,-16},{102,-16}}, color={0,0,127}));
    connect(actuatorBus.openingLPTv, valve_degradation1.open_in) annotation (
        Line(
        points={{30,100},{110,100},{110,7.28}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(LPT2_Bypass.opening, valve_degradation2.open_out)
      annotation (Line(points={{186,16},{200.6,16}}, color={0,0,127}));
    connect(actuatorBus.Divert_Valve_Position, valve_degradation2.open_in)
      annotation (Line(
        points={{30,100},{224,100},{224,16},{211.28,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(tcv_derv_send.y, systemFail_Index.tcv_dervInput) annotation (Line(
          points={{191.2,145},{191.2,142},{200,142},{200,145},{200.96,145}},
          color={0,0,127}));
    connect(ltv1_derv_send.y, systemFail_Index.LTV1_dervInput) annotation (Line(
          points={{191.2,133},{191.2,134},{200.48,134},{200.48,135}},    color={0,
            0,127}));
    connect(ltv2_derv_send.y, systemFail_Index.LTV2_dervInput) annotation (Line(
          points={{191.2,121},{191.2,124},{200.48,124},{200.48,125}},    color={0,
            0,127}));
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
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type5_test;

  model
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type5_Daniel
    "type3 + Valve (MOV) setpoint drift modeling_based on hazard function"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable
        ControlSystems.CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TCVcontrol_CompDegradation_type1
        CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);
    Real tcv_derv;
    Real ltv1_derv;
    Real ltv2_derv;

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{64,122},{84,142}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_HPT,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={238,66})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{244,-62},{224,-42}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      redeclare model EfficiencyChar =
          TRANSFORM.Fluid.Machines.BaseClasses.PumpCharacteristics.Efficiency.Constant,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
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
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));
    Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
      annotation (Placement(transformation(extent={{-106,108},{-86,116}})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT1(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_LPT1,
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15) annotation (Placement(transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=30)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-108,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-19})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
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
      V=5,
      p_start=1500000,
      T_start=423.15)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={178,50})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT2(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_LPT2,
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{90,122},{110,142}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-104,-42},{-84,-22}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={44,-32})));
    Data.ComponentDegradation componentDegradation(
      HPT_lambda=0.00000001,
      LPT1_lambda=0.00000001,
      LPT2_lambda=0.00000001)
      annotation (Placement(transformation(extent={{120,122},{140,142}})));
    Component_Degradation.valve_degradation valve_degradation annotation (
        Placement(transformation(
          extent={{6,-6},{-6,6}},
          rotation=90,
          origin={-4,60})));
    Component_Degradation.valve_degradation valve_degradation1 annotation (
        Placement(transformation(
          extent={{6,-6},{-6,6}},
          rotation=90,
          origin={110,2})));
    Component_Degradation.valve_degradation valve_degradation2 annotation (
        Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=180,
          origin={206,16})));
    Component_Degradation.systemFail_Index systemFail_Index
      annotation (Placement(transformation(extent={{-70,144},{-22,194}})));
    Modelica.Blocks.Sources.RealExpression tcv_derv_send(y=tcv_derv)
      annotation (Placement(transformation(extent={{-104,172},{-80,186}})));
    Modelica.Blocks.Sources.RealExpression ltv1_derv_send(y=ltv1_derv)
      annotation (Placement(transformation(extent={{-104,160},{-80,174}})));
    Modelica.Blocks.Sources.RealExpression ltv2_derv_send(y=ltv2_derv)
      annotation (Placement(transformation(extent={{-104,148},{-80,162}})));
  initial equation

  equation
    port_e.W = generator.power;
    tcv_derv = der(TCV.opening);
    ltv1_derv = der(LPT1_Bypass.opening);
    ltv2_derv = der(LPT2_Bypass.opening);

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{258,100},{258,-98},{-34,-98},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-20,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-30,76},{-80,76},{-80,112},{-85,112}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(
        points={{-10,-58},{-24,-58}},
        color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-30,-44},{-56,-44},{-56,-74},{-108,-74},{-108,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{234,-62},
            {234,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-26},{242,-42},{241,-42}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,48},{-46,
            48},{-46,72},{-66,72}},
                                color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,48},{-34,48},{-34,40},{-12,40}},
                                                   color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,48},{
            -34,48},{-34,62},{-14,62},{-14,66}},
                                         color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-140,48},{-68,48}}, color={0,127,255}));
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
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-12}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-58},{2,-58}},                     color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={{94,-26},
            {94,-32},{54,-32}},          color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{34,-32},{-84,-32}},color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-30,-42},{44,-42},{44,-35.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(generator.shaft, LPT2.shaft_b) annotation (Line(points={{238.1,55.9},
            {238.1,34},{218,34}}, color={0,0,0}));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-98,-58},{-44,-58}}, color={0,127,255}));
    connect(TCV.opening, valve_degradation.open_out)
      annotation (Line(points={{-4,46.4},{-4,54.6}}, color={0,0,127}));
    connect(actuatorBus.opening_TCV, valve_degradation.open_in) annotation (
        Line(
        points={{30.1,100.1},{-4,100.1},{-4,65.28}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
    connect(valve_degradation1.open_out, LPT1_Bypass.opening) annotation (Line(
          points={{110,-3.4},{110,-16},{102,-16}}, color={0,0,127}));
    connect(actuatorBus.openingLPTv, valve_degradation1.open_in) annotation (
        Line(
        points={{30,100},{110,100},{110,7.28}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(LPT2_Bypass.opening, valve_degradation2.open_out)
      annotation (Line(points={{186,16},{200.6,16}}, color={0,0,127}));
    connect(actuatorBus.Divert_Valve_Position, valve_degradation2.open_in)
      annotation (Line(
        points={{30,100},{224,100},{224,16},{211.28,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(tcv_derv_send.y, systemFail_Index.tcv_dervInput) annotation (Line(
          points={{-78.8,179},{-78.8,176},{-70,176},{-70,179},{-69.04,179}},
          color={0,0,127}));
    connect(ltv1_derv_send.y, systemFail_Index.LTV1_dervInput) annotation (Line(
          points={{-78.8,167},{-78.8,168},{-69.52,168},{-69.52,169}},    color=
            {0,0,127}));
    connect(ltv2_derv_send.y, systemFail_Index.LTV2_dervInput) annotation (Line(
          points={{-78.8,155},{-78.8,158},{-69.52,158},{-69.52,159}},    color=
            {0,0,127}));
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
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type5_Daniel;

  model
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type5
    "type3 + Valve (MOV) setpoint drift modeling_based on hazard function"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable
        ControlSystems.CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TCVcontrol_CompDegradation_type1
        CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);


    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{64,122},{84,142}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_HPT,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={238,66})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{244,-62},{224,-42}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      redeclare model EfficiencyChar =
          TRANSFORM.Fluid.Machines.BaseClasses.PumpCharacteristics.Efficiency.Constant,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
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
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{8,8},{-8,-8}},
          rotation=180,
          origin={-4,40})));
    Modelica.Blocks.Sources.RealExpression Electrical_Power(y=generator.power)
      annotation (Placement(transformation(extent={{-106,108},{-86,116}})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT1(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_LPT1,
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15) annotation (Placement(transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=30)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-108,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-19})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
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
      V=5,
      p_start=1500000,
      T_start=423.15)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={178,50})));
    TRANSFORM.Fluid.Machines.SteamTurbine LPT2(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Degradation_LPT2,
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{90,122},{110,142}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-104,-42},{-84,-22}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={44,-32})));
    Data.ComponentDegradation componentDegradation(
      HPT_lambda=0.00000001,
      LPT1_lambda=0.00000001,
      LPT2_lambda=0.00000001)
      annotation (Placement(transformation(extent={{120,122},{140,142}})));
    Component_Degradation.valve_degradation valve_degradation annotation (
        Placement(transformation(
          extent={{6,-6},{-6,6}},
          rotation=90,
          origin={-4,60})));
    Component_Degradation.valve_degradation valve_degradation1 annotation (
        Placement(transformation(
          extent={{6,-6},{-6,6}},
          rotation=90,
          origin={110,2})));
    Component_Degradation.valve_degradation valve_degradation2 annotation (
        Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=180,
          origin={206,16})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{258,100},{258,-98},{-34,-98},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-20,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-30,76},{-80,76},{-80,112},{-85,112}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(
        points={{-10,-58},{-24,-58}},
        color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-30,-44},{-56,-44},{-56,-74},{-108,-74},{-108,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{234,-62},
            {234,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-26},{242,-42},{241,-42}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,48},{-46,
            48},{-46,72},{-66,72}},
                                color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,48},{-34,48},{-34,40},{-12,40}},
                                                   color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,48},{
            -34,48},{-34,62},{-14,62},{-14,66}},
                                         color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-140,48},{-68,48}}, color={0,127,255}));
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
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-12}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-58},{2,-58}},                     color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={{94,-26},
            {94,-32},{54,-32}},          color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{34,-32},{-84,-32}},color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-30,-42},{44,-42},{44,-35.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(generator.shaft, LPT2.shaft_b) annotation (Line(points={{238.1,55.9},
            {238.1,34},{218,34}}, color={0,0,0}));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-98,-58},{-44,-58}}, color={0,127,255}));
    connect(TCV.opening, valve_degradation.open_out)
      annotation (Line(points={{-4,46.4},{-4,54.6}}, color={0,0,127}));
    connect(actuatorBus.opening_TCV, valve_degradation.open_in) annotation (
        Line(
        points={{30.1,100.1},{-4,100.1},{-4,65.28}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
    connect(valve_degradation1.open_out, LPT1_Bypass.opening) annotation (Line(
          points={{110,-3.4},{110,-16},{102,-16}}, color={0,0,127}));
    connect(actuatorBus.openingLPTv, valve_degradation1.open_in) annotation (
        Line(
        points={{30,100},{110,100},{110,7.28}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(LPT2_Bypass.opening, valve_degradation2.open_out)
      annotation (Line(points={{186,16},{200.6,16}}, color={0,0,127}));
    connect(actuatorBus.Divert_Valve_Position, valve_degradation2.open_in)
      annotation (Line(
        points={{30,100},{224,100},{224,16},{211.28,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
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
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type5;

  model
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type6
    "type4 + new Failure Logic different from Type5 test"
    extends BaseClasses.Partial_SubSystem(
      redeclare replaceable
        ControlSystems.CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TCVcontrol_CompDegradation_type5
        CS,
      redeclare replaceable ControlSystems.ED_Dummy ED,
      redeclare Data.IdealTurbine data);

    PrimaryHeatSystem.HTGR.HTGR_Rankine.Data.DataInitial_HTGR_Pebble dataInitial(
        P_LP_Comp_Ref=4000000)
      annotation (Placement(transformation(extent={{64,122},{84,142}})));

    TRANSFORM.Fluid.Machines.SteamTurbine HPT(
      nUnits=1,
      eta_mech=0.93,
      redeclare model Eta_wetSteam =
          TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.eta_Constant,
      p_a_start=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_b_start=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_a_start=dataInitial_HTGR_BoP_3stage.HPT_T_inlet,
      T_b_start=dataInitial_HTGR_BoP_3stage.HPT_T_outlet,
      m_flow_nominal=200,
      p_inlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_inlet,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.HPT_P_outlet,
      T_nominal=dataInitial_HTGR_BoP_3stage.HPT_T_inlet)
      annotation (Placement(transformation(extent={{34,24},{54,44}})));

    TRANSFORM.Electrical.PowerConverters.Generator_Basic generator
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={238,66})));
    Fluid.Vessels.IdealCondenser Condenser(
      p=10000,
      V_total=2500,
      V_liquid_start=1.2)
      annotation (Placement(transformation(extent={{244,-62},{224,-42}})));
    TRANSFORM.Fluid.Machines.Pump_Controlled pump(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      redeclare model EfficiencyChar =
          TRANSFORM.Fluid.Machines.BaseClasses.PumpCharacteristics.Efficiency.Constant,
      N_nominal=1200,
      dp_nominal=15000000,
      m_flow_nominal=50,
      d_nominal=1000,
      controlType="RPM",
      use_port=true)
      annotation (Placement(transformation(extent={{-24,-48},{-44,-68}})));

    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T1(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{6,6},{-6,-6}},
          rotation=180,
          origin={22,40})));
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
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
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
      p_a_start=3000000,
      p_b_start=1500000,
      T_a_start=573.15,
      T_b_start=473.15,
      m_flow_nominal=200,
      p_inlet_nominal=2000000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT1_P_outlet,
      T_nominal=423.15) annotation (Placement(transformation(
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
          origin={-4,-58})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      V=5,
      p_start=2500000,
      T_start=573.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={94,50})));
    TRANSFORM.Fluid.Valves.ValveLinear LPT1_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=30)
                        annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={94,-16})));
    TRANSFORM.Fluid.Sensors.TemperatureTwoPort
                                         sensor_T2(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-108,-58})));
    TRANSFORM.Fluid.Machines.Pump_PressureBooster
                                             pump1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,
      use_input=false,
      p_nominal=5500000,
      allowFlowReversal=false)
      annotation (Placement(transformation(extent={{40,-92},{20,-72}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{7,-8},{-7,8}},
          rotation=90,
          origin={242,-19})));
    TRANSFORM.Fluid.Valves.ValveLinear TBV(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=50) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=180,
          origin={-74,72})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=12000000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-116,62},{-96,82}})));
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
      V=5,
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
      p_a_start=1500000,
      p_b_start=8000,
      T_a_start=523.15,
      T_b_start=343.15,
      m_flow_nominal=200,
      p_inlet_nominal=1500000,
      p_outlet_nominal=dataInitial_HTGR_BoP_3stage.LPT2_P_outlet,
      T_nominal=423.15) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={208,34})));

    TRANSFORM.Fluid.Valves.ValveLinear LPT2_Bypass(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      dp_nominal=100000,
      m_flow_nominal=5) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={178,16})));
    Data.DataInitial_HTGR_BoP_3stage dataInitial_HTGR_BoP_3stage(LPT1_T_outlet=
          473.15, LPT2_T_inlet=473.15)
      annotation (Placement(transformation(extent={{90,122},{110,142}})));
    StagebyStageTurbineSecondary.StagebyStageTurbine.BaseClasses.TRANSFORMMoistureSeparator_MIKK
      Moisture_Separator2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p_start=2500000,
      T_start=573.15,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume)
      annotation (Placement(transformation(extent={{140,40},{160,60}})));
    TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary2(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=5500000,
      T=573.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-104,-42},{-84,-22}})));
    TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
          Modelica.Media.Water.StandardWater)            annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={44,-32})));
    Data.ComponentDegradation componentDegradation(
      HPT_lambda=0.00000001,
      LPT1_lambda=0.00000001,
      LPT2_lambda=0.00000001)
      annotation (Placement(transformation(extent={{120,122},{140,142}})));
    Component_Degradation.valve_degradation valve_degradation annotation (
        Placement(transformation(
          extent={{6,-6},{-6,6}},
          rotation=90,
          origin={-4,60})));
    Component_Degradation.valve_degradation valve_degradation1 annotation (
        Placement(transformation(
          extent={{6,-6},{-6,6}},
          rotation=90,
          origin={110,2})));
    Component_Degradation.valve_degradation valve_degradation2 annotation (
        Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=180,
          origin={206,16})));
  initial equation

  equation
    port_e.W = generator.power;

    connect(HPT.portHP, sensor_T1.port_b) annotation (Line(
        points={{34,40},{28,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Steam_Temperature, sensor_T1.T) annotation (Line(
        points={{-30,100},{4,100},{4,50},{22,50},{22,42.16}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(actuatorBus.Feed_Pump_Speed, pump.inputSignal) annotation (Line(
        points={{30,100},{258,100},{258,-98},{-34,-98},{-34,-65}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(sensorBus.Steam_Pressure, sensor_p.p) annotation (Line(
        points={{-30,100},{-30,76},{-20,76}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(TCV.port_b, sensor_T1.port_a) annotation (Line(
        points={{4,40},{16,40}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Power, Electrical_Power.y) annotation (Line(
        points={{-30,100},{-30,76},{-80,76},{-80,112},{-85,112}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(volume1.port_b, pump.port_a) annotation (Line(
        points={{-10,-58},{-24,-58}},
        color={0,127,255},
        thickness=0.5));
    connect(LPT1.portHP, tee1.port_1) annotation (Line(
        points={{118,40},{118,50},{104,50}},
        color={0,127,255},
        thickness=0.5));
    connect(tee1.port_3, LPT1_Bypass.port_a) annotation (Line(
        points={{94,40},{94,-6}},
        color={0,127,255},
        thickness=0.5));
    connect(sensorBus.Feedwater_Temp, sensor_T2.T) annotation (Line(
        points={{-30,100},{-30,-44},{-56,-44},{-56,-74},{-108,-74},{-108,-61.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5));
    connect(Condenser.port_b, pump1.port_a) annotation (Line(points={{234,-62},
            {234,-82},{40,-82}},                                      color={0,127,
            255},
        thickness=0.5));
    connect(pump1.port_b, volume1.port_a) annotation (Line(points={{20,-82},{16,
            -82},{16,-58},{2,-58}},                    color={0,127,255},
        thickness=0.5));
    connect(HPT.shaft_b, LPT1.shaft_a) annotation (Line(
        points={{54,34},{118,34}},
        color={0,0,0},
        pattern=LinePattern.Dash));
    connect(sensor_m_flow1.port_b,Condenser. port_a)
      annotation (Line(points={{242,-26},{242,-42},{241,-42}},
                                                       color={0,127,255},
        thickness=0.5));

    connect(TBV.port_b, boundary1.ports[1])
      annotation (Line(points={{-82,72},{-96,72}}, color={0,127,255}));
    connect(volume.port_b, TBV.port_a) annotation (Line(points={{-56,48},{-46,
            48},{-46,72},{-66,72}},
                                color={0,127,255}));
    connect(volume.port_b, TCV.port_a)
      annotation (Line(points={{-56,48},{-34,48},{-34,40},{-12,40}},
                                                   color={0,127,255}));
    connect(volume.port_b, sensor_p.port) annotation (Line(points={{-56,48},{
            -34,48},{-34,62},{-14,62},{-14,66}},
                                         color={0,127,255}));
    connect(port_a, volume.port_a)
      annotation (Line(points={{-140,48},{-68,48}}, color={0,127,255}));
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
    connect(LPT2.portLP, sensor_m_flow1.port_a)
      annotation (Line(points={{218,40},{242,40},{242,-12}}, color={0,127,255}));
    connect(tee2.port_3, LPT2_Bypass.port_a)
      annotation (Line(points={{178,40},{178,26}},color={0,127,255}));
    connect(LPT2_Bypass.port_b, pump1.port_a) annotation (Line(points={{178,6},
            {178,-82},{40,-82}},color={0,127,255}));
    connect(LPT1.portLP, Moisture_Separator2.port_a) annotation (Line(points={{
            138,40},{138,50},{144,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_b, tee2.port_2)
      annotation (Line(points={{156,50},{168,50}}, color={0,127,255}));
    connect(Moisture_Separator2.port_Liquid, volume1.port_a) annotation (Line(
          points={{146,46},{146,-58},{2,-58}},                     color={0,127,
            255}));
    connect(HPT.portLP, tee1.port_2) annotation (Line(points={{54,40},{78,40},{
            78,50},{84,50}}, color={0,127,255}));
    connect(LPT1_Bypass.port_b, sensor_m_flow.port_a) annotation (Line(points={{94,-26},
            {94,-32},{54,-32}},          color={0,127,255}));
    connect(sensor_m_flow.port_b, boundary2.ports[1])
      annotation (Line(points={{34,-32},{-84,-32}},color={0,127,255}));
    connect(sensorBus.massflow_LPTv, sensor_m_flow.m_flow) annotation (Line(
        points={{-30,100},{-30,-42},{44,-42},{44,-35.6}},
        color={239,82,82},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(generator.shaft, LPT2.shaft_b) annotation (Line(points={{238.1,55.9},
            {238.1,34},{218,34}}, color={0,0,0}));
    connect(sensor_T2.port_a, pump.port_b)
      annotation (Line(points={{-98,-58},{-44,-58}}, color={0,127,255}));
    connect(TCV.opening, valve_degradation.open_out)
      annotation (Line(points={{-4,46.4},{-4,54.6}}, color={0,0,127}));
    connect(actuatorBus.opening_TCV, valve_degradation.open_in) annotation (
        Line(
        points={{30.1,100.1},{-4,100.1},{-4,65.28}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
    connect(valve_degradation1.open_out, LPT1_Bypass.opening) annotation (Line(
          points={{110,-3.4},{110,-16},{102,-16}}, color={0,0,127}));
    connect(actuatorBus.openingLPTv, valve_degradation1.open_in) annotation (
        Line(
        points={{30,100},{110,100},{110,7.28}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(LPT2_Bypass.opening, valve_degradation2.open_out)
      annotation (Line(points={{186,16},{200.6,16}}, color={0,0,127}));
    connect(actuatorBus.Divert_Valve_Position, valve_degradation2.open_in)
      annotation (Line(
        points={{30,100},{224,100},{224,16},{211.28,16}},
        color={111,216,99},
        pattern=LinePattern.Dash,
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
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
<p>The goal of the HTGR models is to obtain a baseline functioning model that can be used to investigate HTGR applications within IES. That being the motivation, there are likely incorrect time constants throughout the system without relevant comparative data to use. Note also that the current core model structure, while this loop is described as a pebble bed (prismatic is pending), is still using the old nuclear core geometry file. This is due to some odd modeling failures during attempts to change. I will modify this description should I obtain the correct core functioning with a reasonable geometry. Using the old core geometry to obtain the correct flow values (flow area, hydraulic diameters, Reynolds numbers) should provide accurate-enough information. </p>
<p>The Dittus-Boelter simple correlation for single phase heat transfer in turbulent flow is used to calculate the heat transfer between the fuel and the coolant, and maximum fuel temperatures appear to agree with literature (~1200C). </p>
<p>Separate HTGR models will be developed for different uses. The primary differentiator is whether a combined cycle is going to be integrated or not. The combined cycle thoerized to be used here takes advantage of the relatively hot waste heat that is produced by an HTGR to boil water at low pressure and send that to a turbine. </p>
<p>No part of this HTGR model should be considered to be optimized. Additionally, thermal mass of the system needs references and then will need to be adjusted (likely through pipes replacing current zero-volume volume nodes) to more appropriately reflect system time constants. </p>
</html>"));
  end
    HTGR_Rankine_Cycle_Transient_JY_v1_step10_TCV_Control_PumpDegradation_type6;
end HTGR_RankineCycles;
