within NHES.Systems.BalanceOfPlant.Turbine.ControlSystems;
model
  CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TEST_TCVcontrol_Opti



  extends BaseClasses.Partial_ControlSystem;

  Modelica.Blocks.Sources.Constant const3(k=data.T_Steam_Ref)
    annotation (Placement(transformation(extent={{-72,16},{-52,36}})));
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
    annotation (Placement(transformation(extent={{-40,52},{-20,72}})));
  Modelica.Blocks.Sources.Constant const9(k=data.p_steam_vent)
    annotation (Placement(transformation(extent={{-148,54},{-132,70}})));
  Data.HTGR_Rankine
                  data(p_steam_vent=14000000)
    annotation (Placement(transformation(extent={{-98,-4},{-78,16}})));
  Modelica.Blocks.Sources.ContinuousClock clock(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{48,-38},{68,-18}})));
  Modelica.Blocks.Sources.Constant valvedelay(k=1e6)
    annotation (Placement(transformation(extent={{48,-8},{68,12}})));
  Modelica.Blocks.Logical.Greater greater5
    annotation (Placement(transformation(extent={{88,-8},{108,-28}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV
    annotation (Placement(transformation(extent={{128,-28},{148,-8}})));
  Modelica.Blocks.Sources.Trapezoid trapezoid(
    amplitude=-0.00740122,
    rising=780,
    width=1020,
    falling=780,
    period=3600,
    nperiod=1,
    offset=0.0098683,
    startTime=1e6 + 900)
    annotation (Placement(transformation(extent={{88,16},{108,36}})));
  Modelica.Blocks.Sources.ContinuousClock clock2(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-166,248},{-146,268}})));
  Modelica.Blocks.Sources.Constant valvedelay2(k=6e5)
    annotation (Placement(transformation(extent={{-166,284},{-146,304}})));
  Modelica.Blocks.Logical.Greater greater2
    annotation (Placement(transformation(extent={{-126,284},{-106,264}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV1
    annotation (Placement(transformation(extent={{-86,264},{-66,284}})));
  Modelica.Blocks.Sources.Constant const1(k=-280)
    annotation (Placement(transformation(extent={{-120,294},{-112,302}})));
  Modelica.Blocks.Sources.Constant const2(k=-150)
    annotation (Placement(transformation(extent={{-120,240},{-112,248}})));
  Modelica.Blocks.Sources.Constant const10(k=5000)
    annotation (Placement(transformation(extent={{-64,298},{-56,306}})));
  PrimaryHeatSystem.HTGR.VarLimVarK_PID PID(
    use_k_in=true,
    use_lowlim_in=true,
    use_uplim_in=true,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    with_FF=true,
    k=-5e-1,
    Ti=30) annotation (Placement(transformation(extent={{-4,16},{16,36}})));
  Modelica.Blocks.Sources.Constant const11(k=-1e-1)
    annotation (Placement(transformation(extent={{-120,316},{-112,324}})));
  Modelica.Blocks.Sources.ContinuousClock clock1(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-166,322},{-146,342}})));
  Modelica.Blocks.Sources.Constant valvedelay1(k=8.5e5)
    annotation (Placement(transformation(extent={{-166,358},{-146,378}})));
  Modelica.Blocks.Logical.Greater greater1
    annotation (Placement(transformation(extent={{-126,358},{-106,338}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV2
    annotation (Placement(transformation(extent={{-86,338},{-66,358}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=-0.5e-1,
    duration=1e5,
    offset=-1e-1,
    startTime=8.7e5)
    annotation (Placement(transformation(extent={{-124,378},{-104,398}})));
  Modelica.Blocks.Sources.Trapezoid trapezoid1(
    amplitude=-280,
    rising=780,
    width=1020,
    falling=780,
    period=3600,
    nperiod=1,
    offset=0,
    startTime=1e6 + 900)
    annotation (Placement(transformation(extent={{-150,20},{-130,40}})));
  TRANSFORM.Controls.LimPID LTV1_Divert_Valve1(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=-1e-8,
    Ti=300,
    yMax=1 - 1e-6,
    yMin=0,
    initType=Modelica.Blocks.Types.Init.InitialState,
    xi_start=0.2)
    annotation (Placement(transformation(extent={{-56,112},{-40,128}})));
  Modelica.Blocks.Sources.Constant const_LTV1bypass_power(k=44e6)
    annotation (Placement(transformation(extent={{-148,86},{-132,102}})));
  Modelica.Blocks.Sources.Trapezoid trap_LTV1bypass_massflow(
    amplitude=30,
    rising=5e4,
    width=5e4,
    falling=5e4,
    period=20e4,
    nperiod=-1,
    offset=15,
    startTime=1e5 + 900)
    annotation (Placement(transformation(extent={{-202,112},{-186,128}})));
  Modelica.Blocks.Sources.Ramp ramp_LTV1bypass_massflow(
    height=-15,
    duration=5e4,
    offset=15,
    startTime=1e5 + 900)
    annotation (Placement(transformation(extent={{-202,62},{-186,78}})));
  Modelica.Blocks.Sources.Constant const_LTV1bypass_massflow(k=30)
    annotation (Placement(transformation(extent={{-202,86},{-186,102}})));
  Modelica.Blocks.Sources.Trapezoid trap_LTV1bypass_power(
    amplitude=-10e6,
    rising=3600,
    width=5e4,
    falling=3600,
    period=107200,
    nperiod=-1,
    offset=44e6,
    startTime=2e5)
    annotation (Placement(transformation(extent={{-148,112},{-132,128}})));
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
  Modelica.Blocks.Sources.Constant valvedelay4(k=data.p_steam_vent)
    annotation (Placement(transformation(extent={{-196,-4},{-176,16}})));
  Modelica.Blocks.Math.Add         add1
    annotation (Placement(transformation(extent={{-10,-44},{10,-24}})));
  Modelica.Blocks.Sources.Constant const7(k=1.0)
    annotation (Placement(transformation(extent={{-28,-44},{-20,-36}})));
  Modelica.Blocks.Sources.Constant valvedelay5(k=1e5)
    annotation (Placement(transformation(extent={{-102,174},{-82,194}})));
  Modelica.Blocks.Sources.Constant valvedelay6(k=0.2)
    annotation (Placement(transformation(extent={{-62,188},{-42,208}})));
  Modelica.Blocks.Sources.ContinuousClock clock4(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-102,134},{-82,154}})));
  Modelica.Blocks.Logical.Greater greater4
    annotation (Placement(transformation(extent={{-62,174},{-42,154}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV4
    annotation (Placement(transformation(extent={{-22,154},{-2,174}})));
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
    annotation (Line(points={{-131.2,62},{-42,62}},color={0,0,127}));
  connect(sensorBus.Steam_Pressure, PI_TBV.u_m) annotation (Line(
      points={{-30,-100},{-104,-100},{-104,44},{-30,44},{-30,50}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorBus.TBV, PI_TBV.y) annotation (Line(
      points={{30,-100},{30,62},{-19,62}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(clock.y, greater5.u1) annotation (Line(points={{69,-28},{80,-28},{80,
          -18},{86,-18}}, color={0,0,127}));
  connect(valvedelay.y, greater5.u2) annotation (Line(points={{69,2},{80,2},{80,
          -10},{86,-10}}, color={0,0,127}));
  connect(greater5.y, switch_P_setpoint_TCV.u2)
    annotation (Line(points={{109,-18},{126,-18}}, color={255,0,255}));
  connect(actuatorBus.opening_TCV, switch_P_setpoint_TCV.y) annotation (Line(
      points={{30.1,-99.9},{30.1,-62},{158,-62},{158,-18},{149,-18}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(trapezoid.y, switch_P_setpoint_TCV.u1) annotation (Line(points={{109,
          26},{118,26},{118,-10},{126,-10}}, color={0,0,127}));
  connect(clock2.y, greater2.u1) annotation (Line(points={{-145,258},{-138,258},
          {-138,274},{-128,274}}, color={0,0,127}));
  connect(valvedelay2.y, greater2.u2) annotation (Line(points={{-145,294},{-138,
          294},{-138,282},{-128,282}}, color={0,0,127}));
  connect(greater2.y, switch_P_setpoint_TCV1.u2)
    annotation (Line(points={{-105,274},{-104,274},{-104,280},{-100,280},{-100,
          274},{-88,274}},                          color={255,0,255}));
  connect(const2.y, switch_P_setpoint_TCV1.u3) annotation (Line(points={{-111.6,
          244},{-88,244},{-88,266}},                       color={0,0,127}));
  connect(const1.y, switch_P_setpoint_TCV1.u1) annotation (Line(points={{-111.6,
          298},{-98,298},{-98,282},{-88,282}}, color={0,0,127}));
  connect(actuatorBus.Feed_Pump_Speed, add.y) annotation (Line(
      points={{30,-100},{30,50},{92,50},{92,70},{85,70}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(const10.y, PID.upperlim) annotation (Line(points={{-55.6,302},{0,302},
          {0,37}},                                        color={0,0,127}));
  connect(switch_P_setpoint_TCV1.y, PID.lowerlim) annotation (Line(points={{-65,274},
          {6,274},{6,37}},
        color={0,0,127}));
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
    annotation (Line(points={{-51,26},{-6,26}}, color={0,0,127}));
  connect(PID.y, add.u2) annotation (Line(points={{17,26},{34,26},{34,34},{48,
          34},{48,64},{62,64}}, color={0,0,127}));
  connect(clock1.y, greater1.u1) annotation (Line(points={{-145,332},{-134,332},
          {-134,348},{-128,348}}, color={0,0,127}));
  connect(valvedelay1.y, greater1.u2) annotation (Line(points={{-145,368},{-134,
          368},{-134,356},{-128,356}}, color={0,0,127}));
  connect(greater1.y, switch_P_setpoint_TCV2.u2)
    annotation (Line(points={{-105,348},{-88,348}}, color={255,0,255}));
  connect(const11.y, switch_P_setpoint_TCV2.u3) annotation (Line(points={{-111.6,
          320},{-104,320},{-104,322},{-98,322},{-98,340},{-88,340}},
        color={0,0,127}));
  connect(switch_P_setpoint_TCV2.y, PID.prop_k) annotation (Line(points={{-65,348},
          {14,348},{14,37.4},{13.4,37.4}},                          color={0,0,
          127}));
  connect(ramp.y, switch_P_setpoint_TCV2.u1) annotation (Line(points={{-103,388},
          {-96,388},{-96,356},{-88,356}}, color={0,0,127}));
  connect(trapezoid1.y, PID.u_ff) annotation (Line(points={{-129,30},{-78,30},{
          -78,40},{-12,40},{-12,34},{-6,34}}, color={0,0,127}));
  connect(sensorBus.Power, LTV1_Divert_Valve1.u_m) annotation (Line(
      points={{-30,-100},{-104,-100},{-104,100},{-48,100},{-48,110.4}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(trap_LTV1bypass_power.y, LTV1_Divert_Valve1.u_s)
    annotation (Line(points={{-131.2,120},{-57.6,120}}, color={0,0,127}));
  connect(RPM_TEST.y, add.u1) annotation (Line(points={{50.4,94},{56,94},{56,76},
          {62,76}}, color={0,0,127}));
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
  connect(add1.y, switch_P_setpoint_TCV.u3) annotation (Line(points={{11,-34},{
          44,-34},{44,-54},{118,-54},{118,-26},{126,-26}}, color={0,0,127}));
  connect(clock4.y, greater4.u1) annotation (Line(points={{-81,144},{-81,162},{
          -72,162},{-72,164},{-64,164}}, color={0,0,127}));
  connect(valvedelay5.y, greater4.u2)
    annotation (Line(points={{-81,184},{-81,172},{-64,172}}, color={0,0,127}));
  connect(LTV1_Divert_Valve1.y, switch_P_setpoint_TCV4.u3) annotation (Line(
        points={{-39.2,120},{-34,120},{-34,156},{-24,156}}, color={0,0,127}));
  connect(valvedelay6.y, switch_P_setpoint_TCV4.u1) annotation (Line(points={{
          -41,198},{-34,198},{-34,172},{-24,172}}, color={0,0,127}));
  connect(greater4.y, switch_P_setpoint_TCV4.u2)
    annotation (Line(points={{-41,164},{-24,164}}, color={255,0,255}));
  connect(actuatorBus.openingLPTv, switch_P_setpoint_TCV4.y) annotation (Line(
      points={{30,-100},{30,164},{-1,164}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
annotation(defaultComponentName="changeMe_CS", Icon(graphics));
end
  CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TEST_TCVcontrol_Opti;
