within NHES.Systems.BalanceOfPlant.Turbine.ControlSystems;
model CS_threeStagedTurbine_HTGR_MultiAction

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
                  data(
    p_steam_vent=14000000,
    T_Steam_Ref=793.15,                       Q_Nom=44e6)
    annotation (Placement(transformation(extent={{-98,-4},{-78,16}})));
  Modelica.Blocks.Sources.Constant const2(k=-150)
    annotation (Placement(transformation(extent={{-64,178},{-56,186}})));
  Modelica.Blocks.Sources.Constant const10(k=5000)
    annotation (Placement(transformation(extent={{-64,196},{-56,204}})));
  PrimaryHeatSystem.HTGR.VarLimVarK_PID PID(
    use_k_in=true,
    use_lowlim_in=true,
    use_uplim_in=true,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    with_FF=true,
    k=-5e-1,
    Ti=30) annotation (Placement(transformation(extent={{-4,16},{16,36}})));
  Modelica.Blocks.Sources.Constant const11(k=-1e-1)
    annotation (Placement(transformation(extent={{-64,214},{-56,222}})));
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
    annotation (Placement(transformation(extent={{-186,58},{-170,74}})));
  Modelica.Blocks.Sources.Trapezoid trap_LTV1bypass_power(
    amplitude=-16e6,
    rising=7200,
    width=36000,
    falling=7200,
    period=86400,
    nperiod=-1,
    offset=44e6,
    startTime=1e5)
    annotation (Placement(transformation(extent={{-218,58},{-202,74}})));
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
    annotation (Placement(transformation(extent={{-144,24},{-128,40}})));
  Modelica.Blocks.Sources.Constant const6(k=40e6)
    annotation (Placement(transformation(extent={{-704,-202},{-686,-184}})));
  Modelica.Blocks.Sources.Constant initialTime(k=data.OpTime + data.initTime)
    annotation (Placement(transformation(extent={{-744,-148},{-724,-128}})));
  Modelica.Blocks.Sources.ContinuousClock clock4(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-744,-188},{-724,-168}})));
  Modelica.Blocks.Logical.Greater greater4
    annotation (Placement(transformation(extent={{-704,-148},{-684,-168}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV4
    annotation (Placement(transformation(extent={{-664,-168},{-644,-148}})));
  PrimaryHeatSystem.HTGR.HTGR_Rankine.MultiSwitch_6 p0to3(indicator=9)
    annotation (Placement(transformation(extent={{-704,-132},{-684,-112}})));
  Modelica.Blocks.Sources.Constant AM3(k=data.OpTime + data.initTime + 10800)
    annotation (Placement(transformation(extent={{-682,-32},{-662,-12}})));
  Modelica.Blocks.Sources.ContinuousClock clock1(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-682,-70},{-662,-50}})));
  Modelica.Blocks.Logical.Greater greater1
    annotation (Placement(transformation(extent={{-644,-36},{-624,-56}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV1
    annotation (Placement(transformation(extent={{-604,-56},{-584,-36}})));
  PrimaryHeatSystem.HTGR.HTGR_Rankine.MultiSwitch_6 p3to6(indicator=2)
    annotation (Placement(transformation(extent={{-644,-20},{-624,0}})));
  Modelica.Blocks.Sources.Constant AM6(k=data.OpTime + data.initTime + 21600)
    annotation (Placement(transformation(extent={{-644,60},{-624,80}})));
  Modelica.Blocks.Sources.ContinuousClock clock2(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-644,20},{-624,40}})));
  Modelica.Blocks.Logical.Greater greater2
    annotation (Placement(transformation(extent={{-604,10},{-584,-10}})));
  PrimaryHeatSystem.HTGR.HTGR_Rankine.MultiSwitch_6 p6to9(indicator=3)
    annotation (Placement(transformation(extent={{-604,26},{-584,46}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV2
    annotation (Placement(transformation(extent={{-564,-10},{-544,10}})));
  Modelica.Blocks.Sources.Constant AM9(k=data.OpTime + data.initTime + 32400)
    annotation (Placement(transformation(extent={{-604,102},{-584,122}})));
  Modelica.Blocks.Sources.ContinuousClock clock5(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-604,62},{-584,82}})));
  Modelica.Blocks.Logical.Greater greater5
    annotation (Placement(transformation(extent={{-564,52},{-544,32}})));
  PrimaryHeatSystem.HTGR.HTGR_Rankine.MultiSwitch_6 p9to12(indicator=11)
    annotation (Placement(transformation(extent={{-564,68},{-544,88}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV5
    annotation (Placement(transformation(extent={{-524,32},{-504,52}})));
  Modelica.Blocks.Sources.Constant PM12(k=data.OpTime + data.initTime + 43200)
    annotation (Placement(transformation(extent={{-564,150},{-544,170}})));
  Modelica.Blocks.Sources.ContinuousClock clock6(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-564,110},{-544,130}})));
  PrimaryHeatSystem.HTGR.HTGR_Rankine.MultiSwitch_6 p12to15(indicator=11)
    annotation (Placement(transformation(extent={{-524,116},{-504,136}})));
  Modelica.Blocks.Logical.Greater greater6
    annotation (Placement(transformation(extent={{-524,100},{-504,80}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV6
    annotation (Placement(transformation(extent={{-484,80},{-464,100}})));
  Modelica.Blocks.Sources.Constant PM15(k=data.OpTime + data.initTime + 54000)
    annotation (Placement(transformation(extent={{-524,196},{-504,216}})));
  Modelica.Blocks.Sources.ContinuousClock clock7(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-524,156},{-504,176}})));
  PrimaryHeatSystem.HTGR.HTGR_Rankine.MultiSwitch_6 p15to18(indicator=10)
    annotation (Placement(transformation(extent={{-484,162},{-464,182}})));
  Modelica.Blocks.Logical.Greater greater7
    annotation (Placement(transformation(extent={{-484,146},{-464,126}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV7
    annotation (Placement(transformation(extent={{-444,126},{-424,146}})));
  Modelica.Blocks.Sources.Constant PM18(k=data.OpTime + data.initTime + 64800)
    annotation (Placement(transformation(extent={{-484,236},{-464,256}})));
  Modelica.Blocks.Sources.ContinuousClock clock8(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-484,196},{-464,216}})));
  PrimaryHeatSystem.HTGR.HTGR_Rankine.MultiSwitch_6 p18to21(indicator=2)
    annotation (Placement(transformation(extent={{-444,202},{-424,222}})));
  Modelica.Blocks.Logical.Greater greater8
    annotation (Placement(transformation(extent={{-444,186},{-424,166}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV8
    annotation (Placement(transformation(extent={{-404,166},{-384,186}})));
  Modelica.Blocks.Sources.Constant PM21(k=data.OpTime + data.initTime + 75600)
    annotation (Placement(transformation(extent={{-444,278},{-424,298}})));
  Modelica.Blocks.Sources.ContinuousClock clock9(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-444,238},{-424,258}})));
  PrimaryHeatSystem.HTGR.HTGR_Rankine.MultiSwitch_6 p21to24(indicator=8)
    annotation (Placement(transformation(extent={{-404,244},{-384,264}})));
  Modelica.Blocks.Logical.Greater greater9
    annotation (Placement(transformation(extent={{-404,228},{-384,208}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV9
    annotation (Placement(transformation(extent={{-366,208},{-346,228}})));
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
  connect(actuatorBus.Feed_Pump_Speed, add.y) annotation (Line(
      points={{30,-100},{30,50},{92,50},{92,70},{85,70}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(const10.y, PID.upperlim) annotation (Line(points={{-55.6,200},{0,200},
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
    annotation (Line(points={{-51,26},{-6,26}}, color={0,0,127}));
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
  connect(constant_0.y, PID.u_ff) annotation (Line(points={{-127.2,32},{-76,32},
          {-76,40},{-14,40},{-14,34},{-6,34}}, color={0,0,127}));
  connect(initialTime.y,greater4. u2) annotation (Line(points={{-723,-138},{
          -716,-138},{-716,-150},{-706,-150}},
                                      color={0,0,127}));
  connect(clock4.y,greater4. u1) annotation (Line(points={{-723,-178},{-716,
          -178},{-716,-158},{-706,-158}},
                                  color={0,0,127}));
  connect(const6.y, switch_P_setpoint_TCV4.u3) annotation (Line(points={{-685.1,
          -193},{-666,-193},{-666,-166}}, color={0,0,127}));
  connect(greater4.y,switch_P_setpoint_TCV4. u2)
    annotation (Line(points={{-683,-158},{-666,-158}},
                                                     color={255,0,255}));
  connect(switch_P_setpoint_TCV4.u1, p0to3.y) annotation (Line(points={{-666,
          -150},{-676,-150},{-676,-122},{-683,-122}},
                                                  color={0,0,127}));
  connect(const2.y, PID.lowerlim) annotation (Line(points={{-55.6,182},{2,182},
          {2,46},{6,46},{6,37}}, color={0,0,127}));
  connect(const11.y, PID.prop_k) annotation (Line(points={{-55.6,218},{6,218},{
          6,48},{13.4,48},{13.4,37.4}}, color={0,0,127}));
  connect(AM3.y, greater1.u2) annotation (Line(points={{-661,-22},{-661,-38},{
          -646,-38}},            color={0,0,127}));
  connect(clock1.y,greater1. u1) annotation (Line(points={{-661,-60},{-656,-60},
          {-656,-46},{-646,-46}}, color={0,0,127}));
  connect(greater1.y,switch_P_setpoint_TCV1. u2)
    annotation (Line(points={{-623,-46},{-606,-46}}, color={255,0,255}));
  connect(switch_P_setpoint_TCV1.u1, p3to6.y) annotation (Line(points={{-606,
          -38},{-606,-40},{-616,-40},{-616,-10},{-623,-10}}, color={0,0,127}));
  connect(switch_P_setpoint_TCV4.y, switch_P_setpoint_TCV1.u3) annotation (Line(
        points={{-643,-158},{-643,-160},{-606,-160},{-606,-54}},color={0,0,127}));
  connect(clock2.y, greater2.u1) annotation (Line(points={{-623,30},{-616,30},{
          -616,0},{-606,0}}, color={0,0,127}));
  connect(AM6.y, greater2.u2) annotation (Line(points={{-623,70},{-618,70},{
          -618,68},{-612,68},{-612,20},{-606,20},{-606,8}}, color={0,0,127}));
  connect(greater2.y, switch_P_setpoint_TCV2.u2)
    annotation (Line(points={{-583,0},{-566,0}}, color={255,0,255}));
  connect(switch_P_setpoint_TCV2.u1, p6to9.y) annotation (Line(points={{-566,8},
          {-576,8},{-576,36},{-583,36}}, color={0,0,127}));
  connect(switch_P_setpoint_TCV2.u3, switch_P_setpoint_TCV1.y) annotation (Line(
        points={{-566,-8},{-576,-8},{-576,-46},{-583,-46}}, color={0,0,127}));
  connect(greater5.u1, clock5.y) annotation (Line(points={{-566,42},{-576,42},{
          -576,72},{-583,72}}, color={0,0,127}));
  connect(greater5.u2, AM9.y) annotation (Line(points={{-566,50},{-576,50},{
          -576,108},{-583,108},{-583,112}}, color={0,0,127}));
  connect(p9to12.y, switch_P_setpoint_TCV5.u1)
    annotation (Line(points={{-543,78},{-526,78},{-526,50}}, color={0,0,127}));
  connect(switch_P_setpoint_TCV5.u2, greater5.y)
    annotation (Line(points={{-526,42},{-543,42}}, color={255,0,255}));
  connect(switch_P_setpoint_TCV5.u3, switch_P_setpoint_TCV2.y) annotation (Line(
        points={{-526,34},{-536,34},{-536,0},{-543,0}}, color={0,0,127}));
  connect(clock6.y, greater6.u1) annotation (Line(points={{-543,120},{-536,120},
          {-536,90},{-526,90}}, color={0,0,127}));
  connect(PM12.y, greater6.u2) annotation (Line(points={{-543,160},{-526,160},{
          -526,98}}, color={0,0,127}));
  connect(p12to15.y, switch_P_setpoint_TCV6.u1) annotation (Line(points={{-503,
          126},{-486,126},{-486,98}}, color={0,0,127}));
  connect(greater6.y, switch_P_setpoint_TCV6.u2)
    annotation (Line(points={{-503,90},{-486,90}}, color={255,0,255}));
  connect(switch_P_setpoint_TCV5.y, switch_P_setpoint_TCV6.u3) annotation (Line(
        points={{-503,42},{-496,42},{-496,82},{-486,82}}, color={0,0,127}));
  connect(greater7.u2, PM15.y) annotation (Line(points={{-486,144},{-494,144},{
          -494,208},{-503,208},{-503,206}}, color={0,0,127}));
  connect(greater7.u1, clock7.y) annotation (Line(points={{-486,136},{-494,136},
          {-494,162},{-503,162},{-503,166}}, color={0,0,127}));
  connect(greater7.y, switch_P_setpoint_TCV7.u2)
    annotation (Line(points={{-463,136},{-446,136}}, color={255,0,255}));
  connect(switch_P_setpoint_TCV7.u1, p15to18.y) annotation (Line(points={{-446,
          144},{-456,144},{-456,172},{-463,172}}, color={0,0,127}));
  connect(switch_P_setpoint_TCV6.y, switch_P_setpoint_TCV7.u3) annotation (Line(
        points={{-463,90},{-446,90},{-446,128}}, color={0,0,127}));
  connect(clock8.y, greater8.u1) annotation (Line(points={{-463,206},{-460,206},
          {-460,204},{-456,204},{-456,176},{-446,176}}, color={0,0,127}));
  connect(greater8.u2, PM18.y) annotation (Line(points={{-446,184},{-448,184},{
          -448,196},{-452,196},{-452,246},{-463,246}}, color={0,0,127}));
  connect(greater8.y, switch_P_setpoint_TCV8.u2)
    annotation (Line(points={{-423,176},{-406,176}}, color={255,0,255}));
  connect(switch_P_setpoint_TCV8.u1, p18to21.y) annotation (Line(points={{-406,
          184},{-416,184},{-416,212},{-423,212}}, color={0,0,127}));
  connect(switch_P_setpoint_TCV8.u3, switch_P_setpoint_TCV7.y) annotation (Line(
        points={{-406,168},{-416,168},{-416,136},{-423,136}}, color={0,0,127}));
  connect(clock9.y, greater9.u1) annotation (Line(points={{-423,248},{-418,248},
          {-418,218},{-406,218}}, color={0,0,127}));
  connect(PM21.y, greater9.u2) annotation (Line(points={{-423,288},{-406,288},{
          -406,226}}, color={0,0,127}));
  connect(greater9.y, switch_P_setpoint_TCV9.u2)
    annotation (Line(points={{-383,218},{-368,218}}, color={255,0,255}));
  connect(switch_P_setpoint_TCV8.y, switch_P_setpoint_TCV9.u3) annotation (Line(
        points={{-383,176},{-372,176},{-372,200},{-376,200},{-376,210},{-368,
          210}}, color={0,0,127}));
  connect(p21to24.y, switch_P_setpoint_TCV9.u1) annotation (Line(points={{-383,
          254},{-368,254},{-368,226}}, color={0,0,127}));
  connect(LTV1_Divert_Valve1.u_s, switch_P_setpoint_TCV9.y) annotation (Line(
        points={{-57.6,120},{-332,120},{-332,218},{-345,218}}, color={0,0,127}));
annotation(defaultComponentName="changeMe_CS", Icon(graphics));
end CS_threeStagedTurbine_HTGR_MultiAction;
