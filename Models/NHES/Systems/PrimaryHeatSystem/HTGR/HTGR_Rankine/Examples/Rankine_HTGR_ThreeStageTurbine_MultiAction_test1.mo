within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples;
model Rankine_HTGR_ThreeStageTurbine_MultiAction_test1
  extends Modelica.Icons.Example;

  Real Thermal_Power_Norm;

  Modelica.Units.SI.MassFlowRate sub1_port_a_mass;     // sub 1
  Modelica.Units.SI.Pressure sub1_port_a_press;        // sub 1
  Modelica.Units.SI.Temperature sub1_port_a_temp;      // sub 1

  Modelica.Units.SI.MassFlowRate sub2_TCV_mass;        // sub 2
  Modelica.Units.SI.Pressure sub2_TCV_press;           // sub 2
  Modelica.Units.SI.Temperature sub2_TCV_temp;         // sub 2

  Modelica.Units.SI.MassFlowRate sub3_HPT_mass;        // sub 3
  Modelica.Units.SI.Entropy sub3_HPT_entropyA;         // sub 3
  Modelica.Units.SI.Entropy sub3_HPT_entropyB;         // sub 3
  Modelica.Units.SI.Enthalpy sub3_HPT_enthalpyA;       // sub 3
  Modelica.Units.SI.Enthalpy sub3_HPT_enthalpyB;       // sub 3

  Modelica.Units.SI.MassFlowRate sub4_T1_mass;         // sub 4
  Modelica.Units.SI.MassFlowRate sub4_LPTV1_mass;      // sub 4
  Modelica.Units.SI.Pressure sub4_LPTV1_press;         // sub 4
  Modelica.Units.SI.Temperature sub4_LPTV1_temp;       // sub 4

  Modelica.Units.SI.MassFlowRate sub5_LPT1_mass;        // sub 5
  Modelica.Units.SI.Entropy sub5_LPT1_entropyA;         // sub 5
  Modelica.Units.SI.Entropy sub5_LPT1_entropyB;         // sub 5
  Modelica.Units.SI.Enthalpy sub5_LPT1_enthalpyA;       // sub 5
  Modelica.Units.SI.Enthalpy sub5_LPT1_enthalpyB;       // sub 5

  Modelica.Units.SI.MassFlowRate sub6_T2_mass;         // sub 6
  Modelica.Units.SI.MassFlowRate sub6_LPTV2_mass;      // sub 6
  Modelica.Units.SI.Pressure sub6_LPTV2_press;         // sub 6
  Modelica.Units.SI.Temperature sub6_LPTV2_temp;       // sub 6

  Modelica.Units.SI.MassFlowRate sub7_LPT2_mass;        // sub 7
  Modelica.Units.SI.Entropy sub7_LPT2_entropyA;         // sub 7
  Modelica.Units.SI.Entropy sub7_LPT2_entropyB;         // sub 7
  Modelica.Units.SI.Enthalpy sub7_LPT2_enthalpyA;       // sub 7
  Modelica.Units.SI.Enthalpy sub7_LPT2_enthalpyB;       // sub 7

  Modelica.Units.SI.MassFlowRate sub8_port_b_mass;     // sub 8
  Modelica.Units.SI.Pressure sub8_port_b_press;        // sub 8
  Modelica.Units.SI.Temperature sub8_port_b_temp;      // sub 8

  Real   healthLevel_HPT;
  Real   healthLevel_LPT1;
  Real   healthLevel_LPT2;

  BalanceOfPlant.Turbine.SteamTurbine_L3_HTGR_MultiAction BOP(
    redeclare
      NHES.Systems.BalanceOfPlant.Turbine.ControlSystems.CS_threeStagedTurbine_HTGR_MultiAction
      CS(data(OpTime=OpTime, initTime=initTime),
      p0to3(indicator=5),
      p3to6(indicator=4),
      p6to9(indicator=3),
      p9to12(indicator=2),
      p12to15(indicator=1),
      p15to18(indicator=2),
      p18to21(indicator=3),
      p21to24(indicator=5)),
    lambda_HPT=3e-10,
    lambda_LPT1=3e-10,
    lambda_LPT2=3e-10,
    HPT(eta_mech=eta_mech_HPT),
    LPT1(eta_mech=eta_mech_LPT1),
    LPT2(eta_mech=eta_mech_LPT2),
    LPT1_Bypass(m_flow_nominal=250),
    boundary2(p=2500000))
    annotation (Placement(transformation(extent={{-6,-10},{62,38}})));
  TRANSFORM.Electrical.Sources.FrequencySource
                                     sinkElec(f=60)
    annotation (Placement(transformation(extent={{98,6},{84,22}})));
  Components.HTGR_PebbleBed_Primary_Loop_STHX hTGR_PebbleBed_Primary_Loop(
      redeclare
      NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.ControlSystems.CS_Rankine_Primary_SS
      CS) annotation (Placement(transformation(extent={{-98,-18},{-40,40}})));
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
  Modelica.Blocks.Logical.Timer timer
    annotation (Placement(transformation(extent={{-132,62},{-112,82}})));
  Modelica.Blocks.Sources.ContinuousClock clock4(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-204,50},{-184,70}})));
  Modelica.Blocks.Sources.Constant initialTime(k=OpTime + initTime)
    annotation (Placement(transformation(extent={{-204,78},{-184,98}})));
  Modelica.Blocks.Logical.Greater greater4
    annotation (Placement(transformation(extent={{-164,82},{-144,62}})));
  parameter Real OpTime=0     "Operational time when making decision (unit seconds)";
  parameter Real initTime=3600*12 "Time needed for initialization";
  parameter Real eta_mech_HPT =0.85
                                   "Mechanical efficiency";
  parameter Real eta_mech_LPT1=0.85
                                   "Mechanical efficiency";
  parameter Real eta_mech_LPT2=0.85
                                   "Mechanical efficiency";
equation
  hTGR_PebbleBed_Primary_Loop.input_steam_pressure =BOP.sensor_p.p;
  Thermal_Power_Norm = hTGR_PebbleBed_Primary_Loop.Thermal_Power.y/2.26177E8;

  sub1_port_a_mass = stateDisplay2.statePort.m_flow;     // sub 1
  sub1_port_a_press = BOP.sensor_p.y;                    // sub 1
  sub1_port_a_temp = stateDisplay2.statePort.T;          // sub 1

  sub2_TCV_mass = BOP.TCV.m_flow;                        // sub 2
  sub2_TCV_press = BOP.HPT.portHP.p;                     // sub 2
  sub2_TCV_temp = BOP.TCV.port_a_T;                      // sub 2

  sub3_HPT_mass = BOP.HPT.m_flow*0.99999;                // sub 3 *
  sub3_HPT_entropyA = BOP.HPT_entropy_a;                 // sub 3
  sub3_HPT_entropyB = BOP.HPT_entropy_b;                 // sub 3
  sub3_HPT_enthalpyA = BOP.HPT.portHP.h_outflow;         // sub 3
  sub3_HPT_enthalpyB = BOP.HPT.portLP.h_outflow;         // sub 3

  sub4_T1_mass = BOP.LPT1.portHP.m_flow + BOP.tee1.port_3.m_flow;  // sub 4
  sub4_LPTV1_mass = -BOP.tee1.port_3.m_flow;                       // sub 4
  sub4_LPTV1_press = BOP.tee1.medium.p*0.99999;                    // sub 4 BOP.tee1.medium.p exists
  sub4_LPTV1_temp = BOP.LPT1_Bypass.port_b_T;                      // sub 4

  sub5_LPT1_mass = BOP.LPT1.m_flow;                      // sub 5
  sub5_LPT1_entropyA = BOP.LPT1_entropy_a;               // sub 5
  sub5_LPT1_entropyB = BOP.LPT1_entropy_b;               // sub 5
  sub5_LPT1_enthalpyA = BOP.tee1.medium.h*0.999999;       // sub 5  BOP.tee1.medium.h exists
  sub5_LPT1_enthalpyB = BOP.tee2.medium.h*0.999999;       // sub 5  BOP.tee2.medium.h exists

  sub6_T2_mass = -BOP.Moisture_Separator2.port_b.m_flow; // sub 6
  sub6_LPTV2_mass = BOP.LPT2_Bypass.port_a.m_flow;       // sub 6
  sub6_LPTV2_press = BOP.tee2.medium.p*0.99999;          // sub 6  BOP.tee2.medium.p
  sub6_LPTV2_temp = BOP.LPT2_Bypass.port_a_T;            // sub 6

  sub7_LPT2_mass = BOP.LPT2.m_flow;                      // sub 7
  sub7_LPT2_entropyA = BOP.LPT2_entropy_a;               // sub 7
  sub7_LPT2_entropyB = BOP.LPT2_entropy_b;               // sub 7
  sub7_LPT2_enthalpyA = BOP.tee2.medium.h*0.999999;       // sub 7  BOP.tee2.medium.h exists
  sub7_LPT2_enthalpyB = BOP.LPT2.h_is*0.99999;           // sub 7

  sub8_port_b_mass = stateDisplay1.statePort.m_flow;     // sub 8
  sub8_port_b_press = BOP.FWCP.port_b.p*0.999999;         // sub 8 *
  sub8_port_b_temp = stateDisplay1.statePort.T;          // sub 8

  healthLevel_HPT   = BOP.HPT.eta_wetSteam.eta;
  healthLevel_LPT1  = BOP.LPT1.eta_wetSteam.eta;
  healthLevel_LPT2  = BOP.LPT2.eta_wetSteam.eta;


  connect(sinkElec.port, BOP.port_e)
    annotation (Line(points={{84,14},{62,14}}, color={255,0,0}));
  connect(hTGR_PebbleBed_Primary_Loop.port_b, stateSensor1.port_a) annotation (
      Line(points={{-40.87,25.21},{-40.87,26},{-30,26}}, color={0,127,255}));
  connect(stateSensor1.port_b, BOP.port_a)
    annotation (Line(points={{-14,26},{-6,26},{-6,25.52}}, color={0,127,255}));
  connect(stateSensor2.port_a, BOP.port_b)
    annotation (Line(points={{-14,0},{-6,0.08}}, color={0,127,255}));
  connect(hTGR_PebbleBed_Primary_Loop.port_a, stateSensor2.port_b) annotation (
      Line(points={{-40.87,1.43},{-40.87,0},{-30,0}}, color={0,127,255}));
  connect(stateSensor1.statePort, stateDisplay2.statePort) annotation (Line(
        points={{-21.96,26.05},{-22,26.05},{-22,45.1}}, color={0,0,0}));
  connect(stateSensor2.statePort, stateDisplay1.statePort)
    annotation (Line(points={{-22.04,0.05},{-22,-21.1}}, color={0,0,0}));
  connect(clock4.y, greater4.u1) annotation (Line(points={{-183,60},{-174,60},{
          -174,72},{-166,72}}, color={0,0,127}));
  connect(initialTime.y, greater4.u2) annotation (Line(points={{-183,88},{-172,
          88},{-172,80},{-166,80}}, color={0,0,127}));
  connect(greater4.y, timer.u)
    annotation (Line(points={{-143,72},{-134,72}}, color={255,0,255}));
  annotation (experiment(
      StopTime=186400,
      Interval=1000,
      Tolerance=0.001,
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
end Rankine_HTGR_ThreeStageTurbine_MultiAction_test1;
