within NHES.Systems.BalanceOfPlant.Turbine.ControlSystems;
model
  CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TCVcontrol_CompDegradation_type4


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
    annotation (Placement(transformation(extent={{-302,192},{-282,212}})));
  Modelica.Blocks.Sources.Constant const_LTV1bypass_power(k=44e6)
    annotation (Placement(transformation(extent={{-222,140},{-202,160}})));
  Modelica.Blocks.Sources.ContinuousClock clock4(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-302,152},{-282,172}})));
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
  Modelica.Blocks.Noise.UniformNoise uniformNoise(
    samplePeriod=1000,
    y_min=0,
    y_max=1)
    annotation (Placement(transformation(extent={{-184,216},{-164,236}})));
  HTGR_RankineCycles.Component_Degradation.valveFailProto
                                  system_failure
    annotation (Placement(transformation(extent={{-146,232},{-126,252}})));
  inner Modelica.Blocks.Noise.GlobalSeed globalSeed
    annotation (Placement(transformation(extent={{104,126},{124,146}})));
  Modelica.Blocks.Logical.Greater greater1
    annotation (Placement(transformation(extent={{-224,270},{-204,250}})));
  Modelica.Blocks.Logical.Switch switch_P_setpoint_TCV1
    annotation (Placement(transformation(extent={{-184,250},{-164,270}})));
  Modelica.Blocks.Sources.CombiTimeTable NoHazardFunction(table=[1,0,0; 1001,0,
        0; 2001,0,0; 3001,0,0; 4001,0,0; 5001,0,0; 6001,0,0; 7001,0,0; 8001,0,0;
        9001,0,0; 10001,0,0; 11001,0,0; 12001,0,0; 13001,0,0; 14001,0,0; 15001,
        0,0; 16001,0,0; 17001,0,0; 18001,0,0; 19001,0,0; 20001,0,0; 21001,0,0;
        22001,0,0; 23001,0,0; 24001,0,0; 25001,0,0; 26001,0,0; 27001,0,0; 28001,
        0,0; 29001,0,0; 30001,0,0; 31001,0,0; 32001,0,0; 33001,0,0; 34001,0,0;
        35001,0,0; 36001,0,0; 37001,0,0; 38001,0,0; 39001,0,0; 40001,0,0; 41001,
        0,0; 42001,0,0; 43001,0,0; 44001,0,0; 45001,0,0; 46001,0,0; 47001,0,0;
        48001,0,0; 49001,0,0; 50001,0,0; 51001,0,0; 52001,0,0; 53001,0,0; 54001,
        0,0; 55001,0,0; 56001,0,0; 57001,0,0; 58001,0,0; 59001,0,0; 60001,0,0;
        61001,0,0; 62001,0,0; 63001,0,0; 64001,0,0; 65001,0,0; 66001,0,0; 67001,
        0,0; 68001,0,0; 69001,0,0; 70001,0,0; 71001,0,0; 72001,0,0; 73001,0,0;
        74001,0,0; 75001,0,0; 76001,0,0; 77001,0,0; 78001,0,0; 79001,0,0; 80001,
        0,0; 81001,0,0; 82001,0,0; 83001,0,0; 84001,0,0; 85001,0,0; 86001,0,0])
    annotation (Placement(transformation(extent={{-224,278},{-204,298}})));
  Modelica.Blocks.Sources.CombiTimeTable CumulHazardFunction_1(table=[1,
        1.76124e-05,1.76124e-05; 1001,0.001043086,0.001060698; 2001,0.000539738,
        0.001600437; 3001,0.000435809,0.002036246; 4001,0.00037967,0.002415916;
        5001,0.000342772,0.002758688; 6001,0.000316,0.003074689; 7001,
        0.000295364,0.003370053; 8001,0.000278791,0.003648844; 9001,0.000265079,
        0.003913923; 10001,0.000253474,0.004167398; 11001,0.000243477,
        0.004410875; 12001,0.000234741,0.004645616; 13001,0.000227016,
        0.004872633; 14001,0.000220118,0.00509275; 15001,0.000213905,
        0.005306655; 16001,0.000208269,0.005514924; 17001,0.000203123,
        0.005718047; 18001,0.0001984,0.005916447; 19001,0.000194044,0.006110491;
        20001,0.000190007,0.006300498; 21001,0.000186252,0.00648675; 22001,
        0.000182748,0.006669498; 23001,0.000179466,0.006848963; 24001,
        0.000176383,0.007025347; 25001,0.000173481,0.007198827; 26001,
        0.000170741,0.007369568; 27001,0.000168148,0.007537716; 28001,
        0.00016569,0.007703406; 29001,0.000163355,0.007866761; 30001,
        0.000161133,0.008027894; 31001,0.000159015,0.008186909; 32001,
        0.000156992,0.008343902; 33001,0.000155059,0.00849896; 34001,
        0.000153207,0.008652168; 35001,0.000151432,0.0088036; 36001,0.000149728,
        0.008953328; 37001,0.000148091,0.009101419; 38001,0.000146515,
        0.009247934; 39001,0.000144998,0.009392932; 40001,0.000143536,
        0.009536468; 41001,0.000142125,0.009678593; 42001,0.000140762,
        0.009819355; 43001,0.000139445,0.0099588; 44001,0.000138171,0.01009697;
        45001,0.000136937,0.010233908; 46001,0.000135743,0.01036965; 47001,
        0.000134584,0.010504235; 48001,0.000133461,0.010637695; 49001,
        0.00013237,0.010770065; 50001,0.000131311,0.010901376; 51001,
        0.000130281,0.011031657; 52001,0.00012928,0.011160937; 53001,
        0.000128306,0.011289243; 54001,0.000127358,0.0114166; 55001,0.000126434,
        0.011543034; 56001,0.000125534,0.011668568; 57001,0.000124657,
        0.011793225; 58001,0.000123801,0.011917027; 59001,0.000122967,
        0.012039993; 60001,0.000122152,0.012162145; 61001,0.000121356,
        0.012283501; 62001,0.000120579,0.01240408; 63001,0.000119819,
        0.012523899; 64001,0.000119076,0.012642975; 65001,0.00011835,
        0.012761325; 66001,0.000117639,0.012878964; 67001,0.000116944,
        0.012995908; 68001,0.000116263,0.013112171; 69001,0.000115596,
        0.013227767; 70001,0.000114943,0.01334271; 71001,0.000114303,
        0.013457013; 72001,0.000113676,0.013570689; 73001,0.000113061,
        0.013683751; 74001,0.000112458,0.013796209; 75001,0.000111866,
        0.013908075; 76001,0.000111286,0.014019361; 77001,0.000110716,
        0.014130077; 78001,0.000110157,0.014240235; 79001,0.000109608,
        0.014349842; 80001,0.000109069,0.014458911; 81001,0.000108539,
        0.01456745; 82001,0.000108018,0.014675468; 83001,0.000107507,
        0.014782975; 84001,0.000107004,0.014889979; 85001,0.000106509,
        0.014996488; 86001,0.000106023,0.015102511])
    annotation (Placement(transformation(extent={{-224,216},{-204,236}})));
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
  connect(valvedelay6.y, greater4.u2) annotation (Line(points={{-281,202},{-236,
          202},{-236,128},{-224,128}}, color={0,0,127}));
  connect(clock4.y, greater4.u1) annotation (Line(points={{-281,162},{-240,162},
          {-240,120},{-224,120}}, color={0,0,127}));
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
  connect(uniformNoise.y, system_failure.random) annotation (Line(points={{-163,
          226},{-153,226},{-153,237.8},{-144.8,237.8}}, color={0,0,127}));
  connect(greater1.y, switch_P_setpoint_TCV1.u2)
    annotation (Line(points={{-203,260},{-186,260}}, color={255,0,255}));
  connect(valvedelay6.y, greater1.u2) annotation (Line(points={{-281,202},{-274,
          202},{-274,268},{-226,268}}, color={0,0,127}));
  connect(clock4.y, greater1.u1) annotation (Line(points={{-281,162},{-258,162},
          {-258,260},{-226,260}}, color={0,0,127}));
  connect(NoHazardFunction.y[1], switch_P_setpoint_TCV1.u1) annotation (Line(
        points={{-203,288},{-186,288},{-186,268}}, color={0,0,127}));
  connect(CumulHazardFunction_1.y[1], switch_P_setpoint_TCV1.u3) annotation (
      Line(points={{-203,226},{-188,226},{-188,252},{-186,252}}, color={0,0,127}));
  connect(switch_P_setpoint_TCV1.y, system_failure.hazard) annotation (Line(
        points={{-163,260},{-154,260},{-154,246.6},{-144.8,246.6}}, color={0,0,
          127}));
annotation(defaultComponentName="changeMe_CS", Icon(graphics));
end
  CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TCVcontrol_CompDegradation_type4;
