within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples;
model SteamTurbine_L1_HTGR_v9_notWorking
  import NHES;
  extends Modelica.Icons.Example;
  NHES.Systems.BalanceOfPlant.Turbine.SteamTurbine_L1_boundaries_type2 BOP(
    nPorts_a3=1,
    port_a3_nominal_m_flow={10},
    port_a_nominal(
      m_flow=493.7058,
      p=5550000,
      h=BOP.Medium.specificEnthalpy_pT(BOP.port_a_nominal.p, 591)),
    port_b_nominal(p=1000000, h=BOP.Medium.specificEnthalpy_pT(BOP.port_b_nominal.p,
          318.95)),
    redeclare
      NHES.Systems.BalanceOfPlant.Turbine.ControlSystems.CS_PressureAndPowerControl_HTGRcoupled_v4
      CS(p_nominal=14000000, W_totalSetpoint=sine.y),
    reservoir(level_start=1400))
    annotation (Placement(transformation(extent={{-30,-30},{30,30}})));
  TRANSFORM.Electrical.Sources.FrequencySource
                                     sinkElec(f=60)
    annotation (Placement(transformation(extent={{90,-10},{70,10}})));
  Fluid.Sensors.stateSensor stateSensor(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-66,-22},{-86,-2}})));
  Fluid.Sensors.stateSensor stateSensor1(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-60,2},{-40,22}})));
  Fluid.Sensors.stateDisplay stateDisplay
    annotation (Placement(transformation(extent={{-98,-74},{-54,-44}})));
  Fluid.Sensors.stateDisplay stateDisplay1
    annotation (Placement(transformation(extent={{-72,20},{-28,50}})));
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
      CS) annotation (Placement(transformation(extent={{-150,-30},{-92,26}})));
  TRANSFORM.Fluid.Machines.Pump_Controlled feedWaterpump(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    redeclare model EfficiencyChar =
        TRANSFORM.Fluid.Machines.BaseClasses.PumpCharacteristics.Efficiency.Constant,
    N_nominal=1200,
    dp_nominal=15000000,
    m_flow_nominal=50,
    d_nominal=1000,
    controlType="RPM",
    use_port=true)
    annotation (Placement(transformation(extent={{-40,-2},{-60,-22}})));

  Modelica.Blocks.Sources.Constant const2(k=-1e-1)
    annotation (Placement(transformation(extent={{108,10},{124,26}})));
  Modelica.Blocks.Sources.Constant const10(k=5000)
    annotation (Placement(transformation(extent={{108,-16},{124,0}})));
  Modelica.Blocks.Sources.Constant const1(k=-150)
    annotation (Placement(transformation(extent={{108,-44},{124,-28}})));
  Modelica.Blocks.Sources.Constant const3(k=140e7)
    annotation (Placement(transformation(extent={{86,-76},{102,-60}})));
  NHES.Systems.PrimaryHeatSystem.HTGR.VarLimVarK_PID
                                        PID(
    use_k_in=true,
    use_lowlim_in=true,
    use_uplim_in=true,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    with_FF=true,
    k=-5e-1,
    Ti=30) annotation (Placement(transformation(extent={{130,-78},{150,-58}})));
  Modelica.Blocks.Math.Add         add
    annotation (Placement(transformation(extent={{184,-64},{204,-44}})));
  Modelica.Blocks.Sources.Constant const4(k=1200)
    annotation (Placement(transformation(extent={{152,-46},{168,-30}})));
  Modelica.Blocks.Sources.RealExpression Q_balance(y=stateDisplay1.p)
    "Heat loss/gain not accounted for in connections (e.g., energy vented to atmosphere) [W]"
    annotation (Placement(transformation(extent={{104,-104},{130,-88}})));
  Modelica.Blocks.Sources.Constant const5(k=0)
    annotation (Placement(transformation(extent={{86,-50},{102,-34}})));
equation
  hTGR_PebbleBed_Primary_Loop.input_steam_pressure = BOP.pressure.p;
  // hTGR_PebbleBed_Primary_Loop.input_steam_pressure = 140e7;
  connect(stateDisplay1.statePort, stateSensor1.statePort) annotation (Line(
        points={{-50,31.1},{-50,31.1},{-50,12.05},{-49.95,12.05}}, color={0,0,0}));
  connect(stateDisplay.statePort, stateSensor.statePort) annotation (Line(
        points={{-76,-62.9},{-76,-11.95},{-76.05,-11.95}}, color={0,0,0}));
  connect(stateSensor1.port_b, BOP.port_a)
    annotation (Line(points={{-40,12},{-30,12}}, color={0,127,255}));
  connect(source1.ports[1], BOP.port_a3[1]) annotation (Line(points={{-20,-80},
          {-12,-80},{-12,-30}}, color={0,127,255}));
  connect(BOP.portElec_b, sinkElec.port)
    annotation (Line(points={{30,0},{70,0}}, color={255,0,0}));
  connect(hTGR_PebbleBed_Primary_Loop.port_b, stateSensor1.port_a) annotation (
      Line(points={{-92.87,11.72},{-92.87,12},{-60,12}}, color={0,127,255}));
  connect(stateSensor.port_b, hTGR_PebbleBed_Primary_Loop.port_a) annotation (
      Line(points={{-86,-12},{-92.87,-12},{-92.87,-11.24}}, color={0,127,255}));
  connect(BOP.port_b, feedWaterpump.port_a)
    annotation (Line(points={{-30,-12},{-40,-12}}, color={0,127,255}));
  connect(feedWaterpump.port_b, stateSensor.port_a)
    annotation (Line(points={{-60,-12},{-66,-12}}, color={0,127,255}));
  connect(const2.y, PID.prop_k) annotation (Line(points={{124.8,18},{146,18},{
          146,-56.6},{147.4,-56.6}},
                                 color={0,0,127}));
  connect(const10.y, PID.upperlim)
    annotation (Line(points={{124.8,-8},{134,-8},{134,-57}}, color={0,0,127}));
  connect(const1.y, PID.lowerlim) annotation (Line(points={{124.8,-36},{140,-36},
          {140,-57}}, color={0,0,127}));
  connect(const4.y, add.u1) annotation (Line(points={{168.8,-38},{174,-38},{174,
          -48},{182,-48}}, color={0,0,127}));
  connect(PID.y, add.u2) annotation (Line(points={{151,-68},{174,-68},{174,-60},
          {182,-60}}, color={0,0,127}));
  connect(add.y, feedWaterpump.inputSignal) annotation (Line(points={{205,-54},
          {214,-54},{214,-118},{-50,-118},{-50,-19}},color={0,0,127}));
  connect(Q_balance.y, PID.u_m) annotation (Line(points={{131.3,-96},{140,-96},
          {140,-80}},color={0,0,127}));
  connect(const3.y, PID.u_s)
    annotation (Line(points={{102.8,-68},{128,-68}}, color={0,0,127}));
  connect(const5.y, PID.u_ff) annotation (Line(points={{102.8,-42},{102.8,-56},
          {118,-56},{118,-60},{128,-60}}, color={0,0,127}));
  annotation (experiment(
      StopTime=200,
      __Dymola_NumberOfIntervals=10,
      __Dymola_Algorithm="Dassl"));
end SteamTurbine_L1_HTGR_v9_notWorking;
