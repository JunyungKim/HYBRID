within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples;
model Rankine_HTGR_ThreeStageTurbine_MultiAction_6decisions_final_final
  extends Modelica.Icons.Example;

  Real Thermal_Power_Norm;
  Modelica.Units.SI.Time Time_shift;

  Real   opMode;
  Modelica.Units.SI.Pressure port_a_press;        // sub 1 energy
  Modelica.Units.SI.Temperature port_a_temp;      // sub 1 energy
  Real   TBVmassCum;                              // sub 1 mass (kg)

  Real     healthLevel_Turb;                      // sub 2 health
  Real turbine_massCum;                           // sub 2 mass
  // Modelica.Units.SI.MassFlowRate HPT_mass;
  Modelica.Units.SI.Entropy HPT_entropyA;         // sub 2 energy
  Modelica.Units.SI.Entropy HPT_entropyB;         // sub 2 energy
  Modelica.Units.SI.Enthalpy HPT_enthalpyA;       // sub 2 energy
  Modelica.Units.SI.Enthalpy HPT_enthalpyB;       // sub 2 energy
  Modelica.Units.SI.Temperature   HPT_outletTemp; // sub 2 energy
  Modelica.Units.SI.Temperature   HPT_outletPres; // sub 2 energy
  // Modelica.Units.SI.MassFlowRate LPT1_mass;
  Modelica.Units.SI.Entropy LPT1_entropyA;          // sub 2 energy
  Modelica.Units.SI.Entropy LPT1_entropyB;          // sub 2 energy
  Modelica.Units.SI.Enthalpy LPT1_enthalpyA;        // sub 2 energy
  Modelica.Units.SI.Enthalpy LPT1_enthalpyB;        // sub 2 energy
  Modelica.Units.SI.Temperature   LPT1_outletTemp;  // sub 2 energy
  Modelica.Units.SI.Temperature   LPT1_outletPres;  // sub 2 energy
  // Modelica.Units.SI.MassFlowRate LPT2_mass;
  Modelica.Units.SI.Entropy LPT2_entropyA;            // sub 2 energy
  Modelica.Units.SI.Entropy LPT2_entropyB;            // sub 2 energy
  Modelica.Units.SI.Enthalpy LPT2_enthalpyA;          // sub 2 energy
  Modelica.Units.SI.Enthalpy LPT2_enthalpyB;          // sub 2 energy
  Modelica.Units.SI.Temperature   LPT2_outletTemp;    // sub 2 energy
  Modelica.Units.SI.Temperature   LPT2_outletPres;    // sub 2 energy

  // Modelica.Units.SI.MassFlowRate LPTV1_mass;
  Real   LPTV1_massCum;                           // sub 3 mass
  Modelica.Units.SI.Pressure LPTV1_press;         // sub 3 energy
  Modelica.Units.SI.Temperature LPTV1_temp;       // sub 3 energy

  Modelica.Units.SI.MassFlowRate LPTV2_mass;      // sub 4 massflow
  Modelica.Units.SI.Pressure LPTV2_press;         // sub 4 energy
  Modelica.Units.SI.Temperature LPTV2_temp;       // sub 4 energy

  Modelica.Units.SI.MassFlowRate port_b_mass;     // sub 5 mass
  Modelica.Units.SI.Pressure port_b_press;        // sub 5 energy
  Modelica.Units.SI.Temperature port_b_temp;      // sub 5 energy

  Real electCum_MWh;                               // sub 6
  Modelica.Units.SI.Power elect_MW;                // sub 6

  parameter Real lambda_turbine = 7e-10;


  //parameter Real OpTime=0     "Operational time when making decision (unit seconds)";
  //parameter Real OpTime=0     "Operational time when making decision (unit seconds)";
  parameter Real OpTime=315360000     "10 Yr Operational time when making decision (unit seconds)";
  //parameter Real OpTime=473040000     "15 Yr Operational time when making decision (unit seconds)";
  //parameter Real OpTime=630720000     "20 Yr Operational time when making decision (unit seconds)";

  parameter Real initTime=14400*5     "Time needed for initialization";
  parameter Real eta_mech_HPT =0.85
                                   "Mechanical efficiency";
  parameter Real eta_mech_LPT1=0.85
                                   "Mechanical efficiency";
  parameter Real eta_mech_LPT2=0.85
                                   "Mechanical efficiency";

  BalanceOfPlant.Turbine.SteamTurbine_L3_HTGR_MultiAction_genChanged_final
                                                          BOP(
    redeclare
      NHES.Systems.BalanceOfPlant.Turbine.ControlSystems.CS_threeStagedTurbine_HTGR_MultiAction_6Decisions
      CS(
      data(OpTime=OpTime, initTime=initTime),
      p0to4(indicator=4),
      const6(k=32e6),
      p4to8(indicator=6),
      p8to12(indicator=5),
      p12to16(indicator=6),
      p16to20(indicator=3),
      LTV1_Divert_Valve1(yMin=1e-6),
      p20to24(indicator=1)),
    lambda_HPT=lambda_turbine,
    lambda_LPT1=lambda_turbine,
    lambda_LPT2=lambda_turbine,
    HPT(eta_mech=eta_mech_HPT),
    LPT1(eta_mech=eta_mech_LPT1),
    LPT2(eta_mech=eta_mech_LPT2),
    LPT1_Bypass(m_flow_nominal=100),
    boundary2(p=4000000),
    TBV(m_flow_nominal=50))
    annotation (Placement(transformation(extent={{-6,-62},{62,-14}})));
  TRANSFORM.Electrical.Sources.FrequencySource
                                     sinkElec(f=60)
    annotation (Placement(transformation(extent={{116,-48},{102,-34}})));
  Components.HTGR_PebbleBed_Primary_Loop_STHX hTGR_PebbleBed_Primary_Loop(
      redeclare
      NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.ControlSystems.CS_Rankine_Primary_SS
      CS,
    boundary1(T_in_internal(start=523.15)),
    core(fuelModel(Fuel_kernel(port_b1(T(start={783.2301557224771,783.2301557224771,
                  783.2301557224771,783.2301557224771,783.2301557224771,783.2301557224771,
                  783.2301557224771,783.2301557224771}, displayUnit="degC"))),
          Fuel_kernel_center(port_b1(T(start={656.2495099759531,656.2495099759531,
                  656.2495099759531,656.2495099759531,656.2495099759531,656.2495099759531,
                  656.2495099759531,656.2495099759531}, displayUnit="degC"))))))
          annotation (Placement(transformation(extent={{-98,-70},{-40,-12}})));
  Fluid.Sensors.stateSensor stateSensor1(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-30,-36},{-14,-16}})));
  Fluid.Sensors.stateSensor stateSensor2(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-14,-62},{-30,-42}})));
  Fluid.Sensors.stateDisplay stateDisplay2
    annotation (Placement(transformation(extent={{-42,-18},{-2,12}})));
  Fluid.Sensors.stateDisplay stateDisplay1
    annotation (Placement(transformation(extent={{-42,-62},{-2,-92}})));
  Modelica.Blocks.Logical.Timer timer
    annotation (Placement(transformation(extent={{-36,24},{-20,40}})));
  Modelica.Blocks.Sources.ContinuousClock clock4(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-94,16},{-80,30}})));
  Modelica.Blocks.Sources.Constant initialTime(k=OpTime + initTime)
    annotation (Placement(transformation(extent={{-94,40},{-80,54}})));
  Modelica.Blocks.Logical.Greater greater4
    annotation (Placement(transformation(extent={{-64,40},{-48,24}})));

  Modelica.Blocks.Sources.RealExpression LPTV1SteamMassflowExtracted(y=BOP.LPT1_Bypass.port_a.m_flow)
    annotation (Placement(transformation(extent={{-34,72},{-16,90}})));
  Modelica.Blocks.Continuous.Integrator LPTV1SteamMassCumulative(use_reset=true,
      use_set=false)
    annotation (Placement(transformation(extent={{4,74},{18,88}})));
  Modelica.Blocks.Continuous.Integrator electCum_Joule(use_reset=true, use_set=false)
    annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=90,
        origin={80,44})));
  Electrical.PowerSensor sensorW
    annotation (Placement(transformation(extent={{72,-32},{90,-50}})));
  Modelica.Blocks.Sources.RealExpression TBVSteamMassflowExtracted(y=BOP.TBV.port_a.m_flow)
    annotation (Placement(transformation(extent={{-96,72},{-78,90}})));
  Modelica.Blocks.Continuous.Integrator TBVSteamMassCumulative(use_reset=true,
      use_set=false)
    annotation (Placement(transformation(extent={{-68,74},{-54,88}})));
  Modelica.Blocks.Sources.RealExpression TurbineSteamMassflow(y=BOP.LPT2.m_flow)
    annotation (Placement(transformation(extent={{40,72},{58,90}})));
  Modelica.Blocks.Continuous.Integrator TurbineSteamMassCumulative(use_reset=true,
      use_set=false)
    annotation (Placement(transformation(extent={{78,74},{92,88}})));
equation
  hTGR_PebbleBed_Primary_Loop.input_steam_pressure =BOP.sensor_p.p;
  Thermal_Power_Norm = hTGR_PebbleBed_Primary_Loop.Thermal_Power.y/2.26177E8;

  Time_shift = timer.y;                                  // Calibrated time

  opMode  = 1;
  port_a_press = BOP.sensor_p.y/10;                            // sub 1 energy
  port_a_temp  = stateDisplay2.statePort.T;                    // sub 1 energy
  TBVmassCum   = TBVSteamMassCumulative.y*0.99999;             // sub 1 mass (kg)

  healthLevel_Turb   = lambda_turbine;                         // sub 2 health
  turbine_massCum    = TurbineSteamMassCumulative.y*0.999999;  // sub 2 mass
  //HPT_mass      = BOP.HPT.m_flow*0.99999;
  HPT_entropyA  = BOP.HPT_entropy_a;                           // sub 2 energy
  HPT_entropyB  = BOP.HPT_entropy_b;                           // sub 2 energy
  HPT_enthalpyA = BOP.HPT.portHP.h_outflow/1000;               // sub 2 energy
  HPT_enthalpyB = BOP.HPT.portLP.h_outflow/1000;               // sub 2 energy
  HPT_outletTemp = BOP.HPT.state_b.T*0.9999999;                // sub 2 energy
  HPT_outletPres = BOP.HPT.state_b.p/1000000;                  // sub 2 energy
  //LPT1_mass = BOP.LPT1.m_flow;
  LPT1_entropyA = BOP.LPT1_entropy_a;                          // sub 2 energy
  LPT1_entropyB = BOP.LPT1_entropy_b;                          // sub 2 energy
  LPT1_enthalpyA = BOP.tee1.medium.h/1000;                     // sub 2 energy
  LPT1_enthalpyB = BOP.tee2.medium.h/1000;                     // sub 2 energy
  LPT1_outletTemp = BOP.LPT1.state_b.T*0.9999999;
  LPT1_outletPres = BOP.LPT1.state_b.p/1000000;
  //LPT2_mass = BOP.LPT2.m_flow;
  LPT2_entropyA = BOP.LPT2_entropy_a;                          // sub 2 energy
  LPT2_entropyB = BOP.LPT2_entropy_b;                          // sub 2 energy
  LPT2_enthalpyA = BOP.tee2.medium.h/1000;                     // sub 2 energy
  LPT2_enthalpyB = BOP.LPT2.h_is/1000;                         // sub 2 energy
  LPT2_outletTemp = BOP.LPT2.state_b.T*0.99999;                // sub 2 energy
  LPT2_outletPres = BOP.LPT2.state_b.p/1000000;                // sub 2 energy

  LPTV1_massCum =LPTV1SteamMassCumulative.y*0.99999;                // sub 3 mass
  // LPTV1_mass    = BOP.LPT1_Bypass.port_a.m_flow;
  LPTV1_press   = BOP.tee1.medium.p/1000000;                        // sub 3 energy
  LPTV1_temp    = BOP.LPT1_Bypass.port_b_T*0.99999;                 // sub 3 energy

  LPTV2_mass = BOP.LPT2_Bypass.port_a.m_flow*0.99999; // sub 4  massflow
  LPTV2_press = BOP.tee2.medium.p/1000000;            // sub 4  energy
  LPTV2_temp = BOP.LPT2_Bypass.port_a_T*0.99999;      // sub 4  energy

  port_b_mass  = stateDisplay1.statePort.m_flow*0.99999;    // sub 5 mass
  port_b_press = BOP.FWCP.port_b.p/1000000;                // sub 5 energy
  port_b_temp  = BOP.sensor_T2.T*0.99999;                   // sub 5 energy

  electCum_MWh  = electCum_Joule.y*2.77777777777777778e-10;              // sub 6 Joule is now changed to MWh.
  elect_MW      = BOP.generator.Q_elec/1e6;                              // sub 6

  connect(hTGR_PebbleBed_Primary_Loop.port_b, stateSensor1.port_a) annotation (
      Line(points={{-40.87,-26.79},{-40.87,-26},{-30,-26}},
                                                         color={0,127,255}));
  connect(stateSensor1.port_b, BOP.port_a)
    annotation (Line(points={{-14,-26},{-6,-26},{-6,-26.48}},
                                                           color={0,127,255}));
  connect(stateSensor2.port_a, BOP.port_b)
    annotation (Line(points={{-14,-52},{-6,-51.92}},
                                                 color={0,127,255}));
  connect(hTGR_PebbleBed_Primary_Loop.port_a, stateSensor2.port_b) annotation (
      Line(points={{-40.87,-50.57},{-40.87,-52},{-30,-52}},
                                                      color={0,127,255}));
  connect(stateSensor1.statePort, stateDisplay2.statePort) annotation (Line(
        points={{-21.96,-25.95},{-22,-25.95},{-22,-6.9}},
                                                        color={0,0,0}));
  connect(stateSensor2.statePort, stateDisplay1.statePort)
    annotation (Line(points={{-22.04,-51.95},{-22,-73.1}},
                                                         color={0,0,0}));
  connect(clock4.y, greater4.u1) annotation (Line(points={{-79.3,23},{-76,23},{-76,
          32},{-65.6,32}},     color={0,0,127}));
  connect(initialTime.y, greater4.u2) annotation (Line(points={{-79.3,47},{-76,47},
          {-76,38},{-66,38},{-66,38.4},{-65.6,38.4}},
                                    color={0,0,127}));
  connect(greater4.y, timer.u)
    annotation (Line(points={{-47.2,32},{-37.6,32}},
                                                   color={255,0,255}));
  connect(LPTV1SteamMassflowExtracted.y, LPTV1SteamMassCumulative.u)
    annotation (Line(points={{-15.1,81},{2.6,81}},   color={0,0,127}));
  connect(greater4.y, LPTV1SteamMassCumulative.reset) annotation (Line(points={{-47.2,
          32},{-46,32},{-46,66},{15.2,66},{15.2,72.6}},
                                              color={255,0,255}));
  connect(sensorW.port_b, sinkElec.port) annotation (Line(points={{90,-40.82},{92,
          -40.82},{92,-41},{102,-41}}, color={255,0,0}));
  connect(BOP.port_e, sensorW.port_a)
    annotation (Line(points={{62,-41.84},{62,-41},{72,-41}}, color={255,0,0}));
  connect(sensorW.W, electCum_Joule.u) annotation (Line(points={{81,-32.54},{81,
          -30},{80,-30},{80,36.8}}, color={0,0,127}));
  connect(greater4.y, electCum_Joule.reset) annotation (Line(points={{-47.2,32},
          {-46,32},{-46,47.6},{72.8,47.6}}, color={255,0,255}));
  connect(TBVSteamMassflowExtracted.y, TBVSteamMassCumulative.u)
    annotation (Line(points={{-77.1,81},{-69.4,81}},
                                                   color={0,0,127}));
  connect(greater4.y, TBVSteamMassCumulative.reset) annotation (Line(points={{-47.2,
          32},{-46,32},{-46,72.6},{-56.8,72.6}},        color={255,0,255}));
  connect(TurbineSteamMassflow.y, TurbineSteamMassCumulative.u)
    annotation (Line(points={{58.9,81},{76.6,81}}, color={0,0,127}));
  connect(greater4.y, TurbineSteamMassCumulative.reset) annotation (Line(points
        ={{-47.2,32},{-46,32},{-46,66},{89.2,66},{89.2,72.6}}, color={255,0,255}));
  annotation (experiment(
      StartTime=315360000,
      StopTime=315518400,
      Interval=7200,
      Tolerance=0.01,
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
end Rankine_HTGR_ThreeStageTurbine_MultiAction_6decisions_final_final;
