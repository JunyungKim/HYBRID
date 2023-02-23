within NHES.Systems.PrimaryHeatSystem.SMR_Generic.Examples;
model SMR_Test_3ST_CFWH
  extends Modelica.Icons.Example;

  BalanceOfPlant.Turbine.SteamTurbine_L3_CloseFeedHeat_withPorts_v1
                                                BOP(
    redeclare replaceable
      NHES.Systems.BalanceOfPlant.Turbine.ControlSystems.CS_L3_SMR CS(data(
        Power_nom=80e6,
        HPT_p_in=3450000,
        Tin=579.25,
        Tfeed=421.15,
        mdot_total=70,
        mdot_fh=5.7,
        mdpt_HPFH=5,
        mdot_hpt=65,
        mdot_lpt1=65,
        mdot_lpt2=59.5)))
    annotation (Placement(transformation(extent={{22,0},{98,60}})));
  TRANSFORM.Fluid.BoundaryConditions.Boundary_pT bypassdump(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    p=280000,
    nPorts=1)
    annotation (Placement(transformation(extent={{-40,-60},{-20,-40}})));
  TRANSFORM.Fluid.BoundaryConditions.Boundary_pT steamdump(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    p=3400000,
    nPorts=1)
    annotation (Placement(transformation(extent={{-40,80},{-20,100}})));
  TRANSFORM.Electrical.Sources.FrequencySource boundary
    annotation (Placement(transformation(extent={{140,10},{120,30}})));
  Components.SMR_High_fidelity_no_pump
    nuScale_Tave_enthalpy_Pressurizer_CR(
    port_a_nominal(
      m_flow=90,
      T(displayUnit="degC") = 421.15,
      p=3447380),
    port_b_nominal(
      T(displayUnit="degC") = 579.75,
      h=2997670,
      p=3447380),
    redeclare CS_SMR_highfidelity CS(
      SG_exit_enthalpy=3e6,
      m_setpoint=90.0,
      Q_nom(displayUnit="MW") = 200000000,
      demand=1.0,
      PID_CR(Ti=15)),
    redeclare package Medium = Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-124,-28},{-20,84}})));
equation
  connect(bypassdump.ports[1], BOP.port_b_bypass)
    annotation (Line(points={{-20,-50},{12,-50},{12,30},{22,30}},
                                             color={0,127,255}));
  connect(steamdump.ports[1], BOP.prt_b_steamdump)
    annotation (Line(points={{-20,90},{12,90},{12,51.6},{22,51.6}},
                                               color={0,127,255}));
  connect(BOP.port_a, nuScale_Tave_enthalpy_Pressurizer_CR.port_b) annotation (
      Line(points={{22,44.4},{1.8,44.4},{1.8,44},{-18.4,44}}, color={0,127,255}));
  connect(BOP.port_b, nuScale_Tave_enthalpy_Pressurizer_CR.port_a) annotation (
      Line(points={{22,12.6},{-8,12.6},{-8,17.6},{-18.4,17.6}}, color={0,127,
          255}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},
            {100,100}})),
    experiment(
      StopTime=10000,
      Interval=20,
      __Dymola_Algorithm="Esdirk45a"),
    __Dymola_experimentSetupOutput(events=false),
    Icon(coordinateSystem(extent={{-100,-80},{100,100}})));
end SMR_Test_3ST_CFWH;
