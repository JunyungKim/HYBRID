within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples;
model TestModel
  extends Modelica.Icons.Example;

  Real Thermal_Power_Norm;
  Real Electrical_Power;

  Modelica.Units.SI.MassFlowRate port_a_mass;     // sub 1
  Modelica.Units.SI.Pressure port_a_press;        // sub 1
  Modelica.Units.SI.Temperature port_a_temp;      // sub 1

  Modelica.Units.SI.MassFlowRate TCV_mass;        // sub 2
  Modelica.Units.SI.Pressure TCV_press;           // sub 2
  Modelica.Units.SI.Temperature TCV_temp;         // sub 2

  Modelica.Units.SI.MassFlowRate HPT_mass;        // sub 3
  Modelica.Units.SI.Entropy HPT_entropyA;         // sub 3
  Modelica.Units.SI.Entropy HPT_entropyB;         // sub 3
  Modelica.Units.SI.Enthalpy HPT_enthalpyA;       // sub 3
  Modelica.Units.SI.Enthalpy HPT_enthalpyB;       // sub 3

  Modelica.Units.SI.MassFlowRate T1_mass;         // sub 4
  Modelica.Units.SI.MassFlowRate LPTV1_mass;      // sub 4
  Modelica.Units.SI.Pressure LPTV1_press;         // sub 4
  Modelica.Units.SI.Temperature LPTV1_temp;       // sub 4

  Modelica.Units.SI.MassFlowRate LPT1_mass;        // sub 5
  Modelica.Units.SI.Entropy LPT1_entropyA;         // sub 5
  Modelica.Units.SI.Entropy LPT1_entropyB;         // sub 5
  Modelica.Units.SI.Enthalpy LPT1_enthalpyA;       // sub 5
  Modelica.Units.SI.Enthalpy LPT1_enthalpyB;       // sub 5

  Modelica.Units.SI.MassFlowRate T2_mass;         // sub 6
  Modelica.Units.SI.MassFlowRate LPTV2_mass;      // sub 6
  Modelica.Units.SI.Pressure LPTV2_press;         // sub 6
  Modelica.Units.SI.Temperature LPTV2_temp;       // sub 6

  Modelica.Units.SI.MassFlowRate LPT2_mass;        // sub 7
  Modelica.Units.SI.Entropy LPT2_entropyA;         // sub 7
  Modelica.Units.SI.Entropy LPT2_entropyB;         // sub 7
  Modelica.Units.SI.Enthalpy LPT2_enthalpyA;       // sub 7
  Modelica.Units.SI.Enthalpy LPT2_enthalpyB;       // sub 7

  Modelica.Units.SI.MassFlowRate port_b_mass;     // sub 8
  Modelica.Units.SI.Pressure port_b_press;        // sub 8
  Modelica.Units.SI.Temperature port_b_temp;      // sub 8

  BalanceOfPlant.Turbine.HTGR_RankineCycles.BOP BOP(
    redeclare NHES.Systems.BalanceOfPlant.Turbine.ControlSystems.CS CS,
    lambda_HPT=0.0000000001,
    lambda_LPT1=0.0000000001,
    lambda_LPT2=0.0000000001,
    Strategy_Change_Time=Strategy_Change_Time)
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

  parameter SI.Time Strategy_Change_Time = 128900000 "Operational Strategy Change Time";
  /*  6 month =  15,552,000
     12 month =  31,104,000
     48 month = 124,416,000
     54 month = 139,968,000
     60 month = 155,520,000 */

equation
  hTGR_PebbleBed_Primary_Loop.input_steam_pressure =BOP.sensor_p.p;
  Thermal_Power_Norm    = hTGR_PebbleBed_Primary_Loop.Thermal_Power.y/2.26177E8;

  Electrical_Power =BOP.generator.power;

  port_a_mass = stateDisplay2.statePort.m_flow;     // sub 1
  port_a_press = stateDisplay2.statePort.p;         // sub 1
  port_a_temp = stateDisplay2.statePort.T;          // sub 1

  TCV_mass = BOP.TCV.m_flow;                        // sub 2
  TCV_press = BOP.TCV.port_a.p;                     // sub 2
  TCV_temp = BOP.TCV.port_a_T;                      // sub 2

  HPT_mass = BOP.HPT.m_flow;                        // sub 3
  HPT_entropyA = BOP.HPT_entropy_a;                 // sub 3
  HPT_entropyB = BOP.HPT_entropy_b;                 // sub 3
  HPT_enthalpyA = BOP.HPT.state_a.h;                // sub 3
  HPT_enthalpyB = BOP.HPT.state_b.h;                // sub 3

  T1_mass = BOP.HPT.portLP.m_flow;                  // sub 4
  LPTV1_mass = BOP.LPT1_Bypass.port_a.m_flow;       // sub 4
  LPTV1_press = BOP.LPT1_Bypass.port_a.p;           // sub 4
  LPTV1_temp = BOP.LPT1_Bypass.port_a_T;            // sub 4

  LPT1_mass = BOP.LPT1.m_flow;                      // sub 5
  LPT1_entropyA = BOP.LPT1_entropy_a;               // sub 5
  LPT1_entropyB = BOP.LPT1_entropy_b;               // sub 5
  LPT1_enthalpyA = BOP.LPT1.state_a.h;              // sub 5
  LPT1_enthalpyB = BOP.LPT1.state_b.h;              // sub 5

  T2_mass = BOP.Moisture_Separator2.port_b.m_flow;  // sub 6
  LPTV2_mass = BOP.LPT2_Bypass.port_a.m_flow;       // sub 6
  LPTV2_press = BOP.LPT2_Bypass.port_a.p;           // sub 6
  LPTV2_temp = BOP.LPT2_Bypass.port_a_T;            // sub 6

  LPT2_mass = BOP.LPT2.m_flow;                      // sub 7
  LPT2_entropyA = BOP.LPT2_entropy_a;               // sub 7
  LPT2_entropyB = BOP.LPT2_entropy_b;               // sub 7
  LPT2_enthalpyA = BOP.LPT2.state_a.h;              // sub 7
  LPT2_enthalpyB = BOP.LPT2.state_b.h;              // sub 7

  port_b_mass = stateDisplay1.statePort.m_flow;     // sub 8
  port_b_press = stateDisplay1.statePort.p;         // sub 8
  port_b_temp = stateDisplay1.statePort.T;          // sub 8




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
      StartTime=128563200,
      StopTime=129000000,
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
end TestModel;
