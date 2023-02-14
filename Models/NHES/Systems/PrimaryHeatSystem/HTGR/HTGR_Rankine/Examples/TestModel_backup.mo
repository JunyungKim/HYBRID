within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples;
model TestModel_backup
  extends Modelica.Icons.Example;

  Real Thermal_Power_Norm;
  Real Electrical_Power;

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

  Real healthLevel_TCV;
  Real healthLevel_LPTV1;
  Real healthLevel_LPTV2;

  parameter SI.Time samplePeriod_data = 3600;
  parameter Real SteadyOperation_Power = 44E6 "Steady Operation Power Level";
  parameter SI.Time Strategy_Change_Time = 160704000 "Operational Strategy Change Time";
  /*  6 month =  16,070,400
     12 month =  32,140,800
     48 month = 128,580,000
     54 month = 144,633,600
     60 month = 160,704,000 */

  BalanceOfPlant.Turbine.HTGR_RankineCycles.BOP BOP(
    TCV_againgModel(strChangeTime(k=k)),
    redeclare NHES.Systems.BalanceOfPlant.Turbine.ControlSystems.CS CS(
        StrategyChangeTiming(k=Strategy_Change_Time), soPower(k=
            SteadyOperation_Power)),
    lambda_HPT=0.000000001,
    lambda_LPT1=0.000000001,
    lambda_LPT2=0.000000001,
    Strategy_Change_Time=Strategy_Change_Time,
    LPTV1_againgModel(strChangeTime(k=k)),
    LPTV2_againgModel(strChangeTime(k=k)),
    systemDegradation_Model_Sec_FTOP_Only(
      valveDegradation_Model1(strategyChangeTime=Strategy_Change_Time,
          uniformNoise(samplePeriod=samplePeriod_data)),
      valveDegradation_Model2(strategyChangeTime=Strategy_Change_Time,
          uniformNoise(samplePeriod=samplePeriod_data)),
      valveDegradation_Model3(strategyChangeTime=Strategy_Change_Time,
          uniformNoise(samplePeriod=samplePeriod_data))))
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

  // parameter Real k=BOP.CS.componentDegradation.Strategy_Change_Time
  //   "Constant output value";

  parameter Real k=Strategy_Change_Time "Constant output value";
  parameter SI.Time strategyChangeTime=BOP.systemDegradation_Model_Sec_FTOP_Only.dataValveDegradationModel.strategyChangeTime
    "strategy Change Timing";
  parameter SI.Period samplePeriod=BOP.systemDegradation_Model_Sec_FTOP_Only.valveDegradation_Model1.samplePeriod
    "Period for sampling the raw random numbers";
equation
  hTGR_PebbleBed_Primary_Loop.input_steam_pressure =BOP.sensor_p.p;
  Thermal_Power_Norm    = hTGR_PebbleBed_Primary_Loop.Thermal_Power.y/2.26177E8;

  Electrical_Power =BOP.generator.power;

  sub1_port_a_mass = stateDisplay2.statePort.m_flow;     // sub 1
  sub1_port_a_press = BOP.sensor_p.y;                    // sub 1
  sub1_port_a_temp = stateDisplay2.statePort.T;          // sub 1

  sub2_TCV_mass = BOP.TCV.m_flow;                        // sub 2
  sub2_TCV_press = BOP.HPT.portHP.p;                     // sub 2
  sub2_TCV_temp = BOP.TCV.port_a_T;                      // sub 2

  sub3_HPT_mass = BOP.HPT.m_flow;                        // sub 3 *
  sub3_HPT_entropyA = BOP.HPT_entropy_a;                 // sub 3
  sub3_HPT_entropyB = BOP.HPT_entropy_b;                 // sub 3
  sub3_HPT_enthalpyA = BOP.HPT.portHP.h_outflow;         // sub 3
  sub3_HPT_enthalpyB = BOP.HPT.portLP.h_outflow;         // sub 3

  sub4_T1_mass = BOP.LPT1.portHP.m_flow + BOP.tee1.port_3.m_flow;  // sub 4
  sub4_LPTV1_mass = BOP.tee1.port_3.m_flow;                        // sub 4
  sub4_LPTV1_press = BOP.tee1.medium.p;                            // sub 4 BOP.tee1.medium.p exists
  sub4_LPTV1_temp = BOP.LPT1_Bypass.port_b_T;                      // sub 4

  sub5_LPT1_mass = BOP.LPT1.m_flow;                      // sub 5
  sub5_LPT1_entropyA = BOP.LPT1_entropy_a;               // sub 5
  sub5_LPT1_entropyB = BOP.LPT1_entropy_b;               // sub 5
  sub5_LPT1_enthalpyA = BOP.tee1.medium.h;               // sub 5  BOP.tee1.medium.h exists
  sub5_LPT1_enthalpyB = BOP.tee2.medium.h;               // sub 5  BOP.tee2.medium.h exists

  sub6_T2_mass = BOP.Moisture_Separator2.port_b.m_flow;  // sub 6
  sub6_LPTV2_mass = BOP.LPT2_Bypass.port_a.m_flow;       // sub 6
  sub6_LPTV2_press = BOP.tee2.medium.p;                  // sub 6  BOP.tee2.medium.p
  sub6_LPTV2_temp = BOP.LPT2_Bypass.port_a_T;            // sub 6

  sub7_LPT2_mass = BOP.LPT2.m_flow;                      // sub 7
  sub7_LPT2_entropyA = BOP.LPT2_entropy_a;               // sub 7
  sub7_LPT2_entropyB = BOP.LPT2_entropy_b;               // sub 7
  sub7_LPT2_enthalpyA = BOP.tee2.medium.h;               // sub 7  BOP.tee2.medium.h exists
  sub7_LPT2_enthalpyB = BOP.LPT2.h_is;                   // sub 7

  sub8_port_b_mass = stateDisplay1.statePort.m_flow;     // sub 8
  sub8_port_b_press = BOP.pump.port_b.p;                 // sub 8 *
  sub8_port_b_temp = stateDisplay1.statePort.T;          // sub 8

  healthLevel_TCV   = BOP.TCV_againgModel.TCV_HazardAfterSwitch.y;
  healthLevel_LPTV1 = BOP.LPTV1_againgModel.LPTV1_HazardAfterSwitch.y;
  healthLevel_LPTV2 = BOP.LPTV2_againgModel.LPTV2_HazardAfterSwitch.y;

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
  annotation (experiment(
      StartTime=160517600,
      StopTime=160704000,
      Interval=1000,
      Tolerance=0.001,
      __Dymola_Algorithm="Esdirk45a"), Documentation(info="<html>
<p>This test is effectively the same as the above &quot;Complex&quot; test but split between two models. </p>
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
end TestModel_backup;
