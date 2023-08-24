within NHES.Systems.PrimaryHeatSystem.HTGR.RankineCycle.Examples;
model Rankine_HTGR_ThreeStageTurbine_CFWHextraction_work_old
  extends Modelica.Icons.Example;
  parameter Modelica.Units.SI.AbsolutePressure P_ext=3e5;
  parameter Modelica.Units.SI.AbsolutePressure P_use=2e5;
  parameter Modelica.Units.SI.Density d_ext=1.783341636 "kg/m3";
  parameter Modelica.Units.SI.MassFlowRate m_ext=4;

  Real eta_th "Thermal Cycle Efficiency";

  Models.PebbleBed_PrimaryLoop_STHX hTGR_PebbleBed_Primary_Loop(redeclare
      NHES.Systems.PrimaryHeatSystem.HTGR.RankineCycle.ControlSystems.CS_Rankine_Primary_SS
      CS) annotation (Placement(transformation(extent={{-100,-20},{-40,40}})));
  Fluid.Sensors.stateSensor stateSensor1(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-30,16},{-14,36}})));
  Fluid.Sensors.stateSensor stateSensor2(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-14,-10},{-30,10}})));
  Fluid.Sensors.stateDisplay stateDisplay2
    annotation (Placement(transformation(extent={{-42,34},{-2,64}})));
  Fluid.Sensors.stateDisplay stateDisplay1
    annotation (Placement(transformation(extent={{-42,-10},{-2,-40}})));
  BalanceOfPlant.RankineCycle.Models.SteamTurbine_L3_HPCFWH_HTGR BOP(
    redeclare replaceable
      NHES.Systems.BalanceOfPlant.RankineCycle.ControlSystems.CS_L3_HTGR_extraction_v2
      CS(
      booleanStep1(startTime=0),
      booleanStep(startTime=0),
      booleanStep2(startTime=0),
      redeclare BalanceOfPlant.RankineCycle.Data.Data_L3 data(
        Power_nom=data.Power_nom,
        HPT_p_in=data.HPT_p_in,
        p_dump=data.p_dump,
        p_i1=data.p_i1,
        p_i2=data.p_i2,
        cond_p=data.cond_p,
        Tin=data.Tin,
        Tfeed=data.Tfeed,
        d_HPT_in(displayUnit="kg/m3") = data.d_HPT_in,
        d_LPT1_in(displayUnit="g/cm3") = data.d_LPT1_in,
        d_LPT2_in(displayUnit="kg/m3") = data.d_LPT2_in,
        mdot_total=data.mdot_total,
        mdot_fh=data.mdot_fh,
        mdpt_HPFH=data.mdpt_HPFH,
        mdot_hpt=data.mdot_hpt,
        mdot_lpt1=data.mdot_lpt1,
        mdot_lpt2=data.mdot_lpt2,
        m_ext=data.m_ext,
        eta_t=data.eta_t,
        eta_mech=data.eta_mech,
        eta_p=data.eta_p)),
    redeclare replaceable BalanceOfPlant.RankineCycle.Data.Data_L3_CFWH data(
      Power_nom=data.Power_nom,
      HPT_p_in=data.HPT_p_in,
      p_dump=data.p_dump,
      p_i1=data.p_i1,
      p_i2=data.p_i2,
      cond_p=data.cond_p,
      Tin=data.Tin,
      Tfeed=data.Tfeed,
      d_HPT_in(displayUnit="kg/m3") = data.d_HPT_in,
      d_LPT1_in(displayUnit="g/cm3") = data.d_LPT1_in,
      d_LPT2_in(displayUnit="kg/m3") = data.d_LPT2_in,
      mdot_total=data.mdot_total,
      mdot_fh=data.mdot_fh,
      mdpt_HPFH=data.mdpt_HPFH,
      mdot_hpt=data.mdot_hpt,
      mdot_lpt1=data.mdot_lpt1,
      mdot_lpt2=data.mdot_lpt2,
      m_ext=data.m_ext,
      eta_t=data.eta_t,
      eta_mech=data.eta_mech,
      eta_p=data.eta_p,
      BypassFeedHeater_NTU=2,
      BypassFeedHeater_Q_init=0.5e6),
    OFWH_1(T_start=333.15),
    LPT1_bypass_valve(dp_nominal=P_ext - P_use, m_flow_nominal=m_ext),
    HPT_bypass_valve(m_flow_nominal=data.mdot_fh))
    annotation (Placement(transformation(extent={{60,-20},{120,40}})));
  TRANSFORM.Fluid.BoundaryConditions.Boundary_pT bypassdump(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    p=data.p_use,
    nPorts=1)
    annotation (Placement(transformation(extent={{0,-70},{20,-50}})));
  TRANSFORM.Fluid.BoundaryConditions.Boundary_pT steamdump(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    p=3400000,
    nPorts=1)
    annotation (Placement(transformation(extent={{0,74},{20,94}})));
  TRANSFORM.Electrical.Sources.FrequencySource boundary
    annotation (Placement(transformation(extent={{180,28},{160,48}})));
  BalanceOfPlant.RankineCycle.Data.Data_L3_CFWH_master data(
    Power_nom=80e6,
    HPT_p_in=14000000,
    p_dump=16000000,
    p_i1=P_ext,
    Tin=788.15,
    Tfeed=481.15,
    d_HPT_in(displayUnit="kg/m3") = 43.049187,
    d_LPT1_in(displayUnit="g/cm3") = d_ext,
    d_LPT2_in(displayUnit="kg/m3"),
    mdot_total=50.55,
    mdot_fh=10.6,
    mdot_hpt=39.945,
    mdot_lpt1=39.945,
    mdot_lpt2=35.7553,
    m_ext=m_ext,
    p_use=P_use,
    eta_t=0.9,
    eta_mech=0.99,
    eta_p=0.8)
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  TRANSFORM.Fluid.BoundaryConditions.MassFlowSource_h
                                                 bypassdump1(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    use_m_flow_in=true,
    m_flow=4,
    h=192e3,
    nPorts=1)
    annotation (Placement(transformation(extent={{180,-12},{160,8}})));
  TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
        Modelica.Media.Water.StandardWater) annotation (Placement(
        transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={40,-50})));
equation
  assert(P_ext>=P_use,"Extraction Pressure is below usage pressure");
  hTGR_PebbleBed_Primary_Loop.input_steam_pressure =BOP.TCV.port_a.p;

  eta_th=(-BOP.port_a_elec.W-BOP.pump.W-BOP.pump1.W-BOP.FWCP.W)/hTGR_PebbleBed_Primary_Loop.core.Q_total.y;
  connect(hTGR_PebbleBed_Primary_Loop.port_b, stateSensor1.port_a) annotation (
      Line(points={{-40.9,24.7},{-40.9,26},{-30,26}},    color={0,127,255}));
  connect(hTGR_PebbleBed_Primary_Loop.port_a, stateSensor2.port_b) annotation (
      Line(points={{-40.9,0.1},{-40.9,0},{-30,0}},    color={0,127,255}));
  connect(stateSensor1.statePort, stateDisplay2.statePort) annotation (Line(
        points={{-21.96,26.05},{-22,26.05},{-22,45.1}}, color={0,0,0}));
  connect(stateSensor2.statePort, stateDisplay1.statePort)
    annotation (Line(points={{-22.04,0.05},{-22,-21.1}}, color={0,0,0}));
  connect(steamdump.ports[1],BOP. prt_b_steamdump)
    annotation (Line(points={{20,84},{52,84},{52,40},{60,40}},
                                               color={0,127,255}));
  connect(boundary.port,BOP. port_a_elec)
    annotation (Line(points={{160,38},{130,38},{130,10},{120,10}},
                                               color={255,0,0}));
  connect(BOP.port_a_steam, stateSensor1.port_b) annotation (Line(points={{60,28},
          {58,28},{58,26},{-14,26}},   color={0,127,255}));
  connect(stateSensor2.port_a, BOP.port_b_feed) annotation (Line(points={{-14,0},
          {50,0},{50,-8},{60,-8}},   color={0,127,255}));
  connect(sensor_m_flow.port_a, BOP.port_b_bypass)
    annotation (Line(points={{40,-40},{40,10},{60,10}}, color={0,127,255}));
  connect(sensor_m_flow.port_b, bypassdump.ports[1])
    annotation (Line(points={{40,-60},{20,-60}}, color={0,127,255}));
  connect(bypassdump1.ports[1], BOP.port_a_cond)
    annotation (Line(points={{160,-2},{120,-2}}, color={0,127,255}));
  connect(sensor_m_flow.m_flow, bypassdump1.m_flow_in) annotation (Line(points=
          {{43.6,-50},{184,-50},{184,6},{180,6}}, color={0,0,127}));
  annotation (experiment(
      StopTime=2500000,
      Interval=20,
      __Dymola_Algorithm="Esdirk45a"), Documentation(info="<html>
<p>Test of Pebble_Bed_Three-Stage_Rankine. The simulation should experience transient where external electricity demand is oscilating and control valves are opening and closing corresponding to the required power demand. </p>
<p>The ThreeStaged Turbine BOP model contains four control elements: </p>
<p>1. maintaining steam (steam generator outlet) pressure by using TCV</p>
<p>2. controling amount of electricity generated by using LPTBV1</p>
<p>3. maintaining feedwater temperature by using LPTBV2</p>
<p>4. maintaining steam (steam generator outlet) temperature by controlling feedwater pump RPM</p>
</html>"),
    __Dymola_Commands(executeCall=Design.Experimentation.sweepParameter(
          Design.Internal.Records.ModelDependencySetup(
          Model=
            "NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples.Rankine_HTGR_Test_AR",
          dependencyParameters={Design.Internal.Records.DependencyParameter(
            name="hTGR_PebbleBed_Primary_Loop.CS.const11.k", values=linspace(
            1e-05,
            1e-06,
            6))},
          VariablesToPlot={Design.Internal.Records.VariableToPlot(name=
            "hTGR_PebbleBed_Primary_Loop.CS.CR.y"),
            Design.Internal.Records.VariableToPlot(name=
            "hTGR_PebbleBed_Primary_Loop.core.Q_total.y"),
            Design.Internal.Records.VariableToPlot(name=
            "hTGR_PebbleBed_Primary_Loop.core.fuelModel[2].T_Fuel")},
          integrator=Design.Internal.Records.Integrator(
            startTime=0,
            stopTime=1004200,
            numberOfIntervals=0,
            outputInterval=10,
            method="Esdirk45a",
            tolerance=0.0001,
            fixedStepSize=0)))),
    __Dymola_experimentSetupOutput(events=false));
end Rankine_HTGR_ThreeStageTurbine_CFWHextraction_work_old;
