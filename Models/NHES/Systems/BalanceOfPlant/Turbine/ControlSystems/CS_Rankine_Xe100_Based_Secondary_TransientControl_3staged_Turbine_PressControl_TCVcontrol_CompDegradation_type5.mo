within NHES.Systems.BalanceOfPlant.Turbine.ControlSystems;
model
  CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TCVcontrol_CompDegradation_type5


  extends BaseClasses.Partial_ControlSystem;

  Modelica.Blocks.Sources.Constant const3(k=data.T_Steam_Ref)
    annotation (Placement(transformation(extent={{-48,18},{-32,34}})));
  Modelica.Blocks.Sources.Constant const4(k=1200)
    annotation (Placement(transformation(extent={{42,72},{50,80}})));
  Modelica.Blocks.Math.Add         add
    annotation (Placement(transformation(extent={{64,60},{84,80}})));
  TRANSFORM.Controls.LimPID LTV2_Divert_Valve(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=1e-5,
    Ti=15,
    yMax=1 - 1e-6,
    yMin=0,
    initType=Modelica.Blocks.Types.Init.InitialState,
    xi_start=1500)
    annotation (Placement(transformation(extent={{-64,-72},{-44,-52}})));
  Modelica.Blocks.Sources.Constant const5(k=data.T_Feedwater)
    annotation (Placement(transformation(extent={{-94,-72},{-74,-52}})));
  TRANSFORM.Controls.LimPID TCV_Position(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=-3e-9,
    Ti=360,
    yMax=0,
    yMin=-1,
    initType=Modelica.Blocks.Types.Init.InitialState,
    xi_start=1500)
    annotation (Placement(transformation(extent={{-56,-18},{-36,-38}})));
  Modelica.Blocks.Sources.Constant const8(k=1e-6)
    annotation (Placement(transformation(extent={{-34,-78},{-26,-70}})));
  Modelica.Blocks.Math.Add         add2
    annotation (Placement(transformation(extent={{-10,-78},{10,-58}})));
  StagebyStageTurbineSecondary.Control_and_Distribution.Timer             timer(
      Start_Time=1e-2)
    annotation (Placement(transformation(extent={{-34,-66},{-26,-58}})));
  TRANSFORM.Controls.LimPID PI_TBV(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=-5e-7,
    Ti=500,
    yMax=1.0,
    yMin=0.0,
    initType=Modelica.Blocks.Types.Init.NoInit)
    annotation (Placement(transformation(extent={{-40,72},{-20,92}})));
  Modelica.Blocks.Sources.Constant const9(k=data.p_steam_vent)
    annotation (Placement(transformation(extent={{-72,72},{-52,92}})));
  Data.HTGR_Rankine
                  data(
    p_steam_vent=14000000,
    T_Steam_Ref=788.15,                       Q_Nom=44e6)
    annotation (Placement(transformation(extent={{-158,80},{-138,100}})));
  Modelica.Blocks.Sources.Constant const1(k=-150)
    annotation (Placement(transformation(extent={{-56,142},{-40,158}})));
  Modelica.Blocks.Sources.Constant const10(k=5000)
    annotation (Placement(transformation(extent={{-56,170},{-40,186}})));
  PrimaryHeatSystem.HTGR.VarLimVarK_PID PID(
    use_k_in=true,
    use_lowlim_in=true,
    use_uplim_in=true,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    with_FF=true,
    k=-5e-1,
    Ti=30) annotation (Placement(transformation(extent={{-4,16},{16,36}})));
  TRANSFORM.Controls.LimPID LTV1_Divert_Valve1(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=-1e-8,
    Ti=300,
    yMax=1 - 1e-6,
    yMin=0,
    initType=Modelica.Blocks.Types.Init.InitialState,
    xi_start=0.2)
    annotation (Placement(transformation(extent={{-56,112},{-40,128}})));
  Modelica.Blocks.Sources.Constant RPM_TEST(k=1000)
    annotation (Placement(transformation(extent={{42,90},{50,98}})));
  Modelica.Blocks.Sources.Constant const12(k=data.p_steam_vent)
    annotation (Placement(transformation(extent={{-196,-72},{-178,-54}})));
  Modelica.Blocks.Sources.Constant valvedelay3(k=1e5)
    annotation (Placement(transformation(extent={{-236,-18},{-216,2}})));
  Modelica.Blocks.Sources.ContinuousClock clock3(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-236,-58},{-216,-38}})));
  Modelica.Blocks.Logical.Greater greater3
    annotation (Placement(transformation(extent={{-196,-18},{-176,-38}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV3
    annotation (Placement(transformation(extent={{-156,-38},{-136,-18}})));
  Modelica.Blocks.Sources.Constant valvedelay4(k=14e6)
    annotation (Placement(transformation(extent={{-196,-4},{-176,16}})));
  Modelica.Blocks.Math.Add         add1
    annotation (Placement(transformation(extent={{-10,-44},{10,-24}})));
  Modelica.Blocks.Sources.Constant const7(k=1.0)
    annotation (Placement(transformation(extent={{-28,-44},{-20,-36}})));
  Modelica.Blocks.Sources.Constant constant_0(k=0)
    annotation (Placement(transformation(extent={{-72,40},{-54,58}})));
  Modelica.Blocks.Sources.Constant valvedelay6(k=componentDegradation.Strategy_Change_Time)
    annotation (Placement(transformation(extent={{-258,128},{-238,148}})));
  Modelica.Blocks.Sources.Constant const_LTV1bypass_power(k=44e6)
    annotation (Placement(transformation(extent={{-222,140},{-202,160}})));
  Modelica.Blocks.Sources.ContinuousClock clock4(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-258,88},{-238,108}})));
  Modelica.Blocks.Logical.Greater greater4
    annotation (Placement(transformation(extent={{-222,130},{-202,110}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV4
    annotation (Placement(transformation(extent={{-182,110},{-162,130}})));
  Modelica.Blocks.Sources.Trapezoid Original_trap_LTV1bypass_power(
    amplitude=-10e6,
    rising=3600,
    width=5e4,
    falling=3600,
    period=107200,
    nperiod=-1,
    offset=44e6,
    startTime=2e5)
    annotation (Placement(transformation(extent={{-222,76},{-202,96}})));
  Data.ComponentDegradation componentDegradation(
    Strategy_Change_Time=8e+5,
    HPT_lambda=0.00000001,
    LPT1_lambda=0.00000001,
    LPT2_lambda=0.00000001)
    annotation (Placement(transformation(extent={{-134,80},{-114,100}})));
  Modelica.Blocks.Sources.Constant const2(k=-1e-1)
    annotation (Placement(transformation(extent={{-56,196},{-40,212}})));
equation

  connect(const5.y,LTV2_Divert_Valve. u_s)
    annotation (Line(points={{-73,-62},{-66,-62}},   color={0,0,127}));
  connect(sensorBus.Feedwater_Temp,LTV2_Divert_Valve. u_m) annotation (Line(
      points={{-30,-100},{-54,-100},{-54,-74}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorBus.Divert_Valve_Position, add2.y) annotation (Line(
      points={{30,-100},{30,-68},{11,-68}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(add2.u2, const8.y) annotation (Line(points={{-12,-74},{-25.6,-74}},
                                                                         color=
          {0,0,127}));
  connect(add2.u1, timer.y) annotation (Line(points={{-12,-62},{-25.44,-62}},
                                                                color={0,0,127}));
  connect(LTV2_Divert_Valve.y, timer.u) annotation (Line(points={{-43,-62},{-34.8,
          -62}},                                                     color={0,0,
          127}));
  connect(const9.y, PI_TBV.u_s)
    annotation (Line(points={{-51,82},{-42,82}},   color={0,0,127}));
  connect(sensorBus.Steam_Pressure, PI_TBV.u_m) annotation (Line(
      points={{-30,-100},{-104,-100},{-104,44},{-30,44},{-30,70}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorBus.TBV, PI_TBV.y) annotation (Line(
      points={{30,-100},{30,82},{-19,82}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorBus.Feed_Pump_Speed, add.y) annotation (Line(
      points={{30,-100},{30,50},{92,50},{92,70},{85,70}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(const10.y, PID.upperlim) annotation (Line(points={{-39.2,178},{0,178},
          {0,37}},                                        color={0,0,127}));
  connect(sensorBus.Steam_Temperature, PID.u_m) annotation (Line(
      points={{-30,-100},{-104,-100},{-104,-8},{6,-8},{6,14}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(const3.y, PID.u_s)
    annotation (Line(points={{-31.2,26},{-6,26}},
                                                color={0,0,127}));
  connect(PID.y, add.u2) annotation (Line(points={{17,26},{34,26},{34,34},{48,
          34},{48,64},{62,64}}, color={0,0,127}));
  connect(actuatorBus.openingLPTv,LTV1_Divert_Valve1. y) annotation (Line(
      points={{30,-100},{120,-100},{120,120},{-39.2,120}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(sensorBus.Power, LTV1_Divert_Valve1.u_m) annotation (Line(
      points={{-30,-100},{-104,-100},{-104,100},{-48,100},{-48,110.4}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(sensorBus.Steam_Pressure, TCV_Position.u_m) annotation (Line(
      points={{-30,-100},{-104,-100},{-104,-10},{-46,-10},{-46,-16}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(valvedelay3.y, greater3.u2) annotation (Line(points={{-215,-8},{-212,
          -8},{-212,-20},{-198,-20}}, color={0,0,127}));
  connect(clock3.y, greater3.u1) annotation (Line(points={{-215,-48},{-212,-48},
          {-212,-28},{-198,-28}}, color={0,0,127}));
  connect(const12.y, switch_P_setpoint_TCV3.u3) annotation (Line(points={{
          -177.1,-63},{-166,-63},{-166,-36},{-158,-36}}, color={0,0,127}));
  connect(greater3.y, switch_P_setpoint_TCV3.u2)
    annotation (Line(points={{-175,-28},{-158,-28}}, color={255,0,255}));
  connect(valvedelay4.y, switch_P_setpoint_TCV3.u1) annotation (Line(points={{
          -175,6},{-166,6},{-166,-20},{-158,-20}}, color={0,0,127}));
  connect(switch_P_setpoint_TCV3.y, TCV_Position.u_s)
    annotation (Line(points={{-135,-28},{-58,-28}}, color={0,0,127}));
  connect(TCV_Position.y, add1.u1)
    annotation (Line(points={{-35,-28},{-12,-28}}, color={0,0,127}));
  connect(const7.y, add1.u2)
    annotation (Line(points={{-19.6,-40},{-12,-40}}, color={0,0,127}));
  connect(const4.y, add.u1)
    annotation (Line(points={{50.4,76},{62,76}}, color={0,0,127}));
  connect(actuatorBus.opening_TCV, add1.y) annotation (Line(
      points={{30.1,-99.9},{30.1,-34},{11,-34}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(constant_0.y, PID.u_ff) annotation (Line(points={{-53.1,49},{-52,49},
          {-52,48},{-12,48},{-12,34},{-6,34}}, color={0,0,127}));
  connect(valvedelay6.y, greater4.u2) annotation (Line(points={{-237,138},{-234,
          138},{-234,128},{-224,128}}, color={0,0,127}));
  connect(clock4.y, greater4.u1) annotation (Line(points={{-237,98},{-234,98},{
          -234,120},{-224,120}}, color={0,0,127}));
  connect(Original_trap_LTV1bypass_power.y, switch_P_setpoint_TCV4.u3)
    annotation (Line(points={{-201,86},{-190,86},{-190,112},{-184,112}}, color=
          {0,0,127}));
  connect(const_LTV1bypass_power.y, switch_P_setpoint_TCV4.u1) annotation (Line(
        points={{-201,150},{-190,150},{-190,128},{-184,128}}, color={0,0,127}));
  connect(switch_P_setpoint_TCV4.y, LTV1_Divert_Valve1.u_s)
    annotation (Line(points={{-161,120},{-57.6,120}}, color={0,0,127}));
  connect(greater4.y, switch_P_setpoint_TCV4.u2)
    annotation (Line(points={{-201,120},{-184,120}}, color={255,0,255}));
  connect(const1.y, PID.lowerlim)
    annotation (Line(points={{-39.2,150},{6,150},{6,37}}, color={0,0,127}));
  connect(const2.y, PID.prop_k) annotation (Line(points={{-39.2,204},{14,204},{
          14,37.4},{13.4,37.4}}, color={0,0,127}));
annotation(defaultComponentName="changeMe_CS", Icon(graphics));
end
  CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TCVcontrol_CompDegradation_type5;
