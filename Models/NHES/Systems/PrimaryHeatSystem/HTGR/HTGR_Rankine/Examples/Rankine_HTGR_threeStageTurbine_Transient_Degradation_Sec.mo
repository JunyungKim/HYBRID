within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples;
model Rankine_HTGR_threeStageTurbine_Transient_Degradation_Sec
  extends Modelica.Icons.Example;

  Real Thermal_Power_Norm;
  Real Electrical_Power_Norm;
  BalanceOfPlant.Turbine.HTGR_RankineCycles.HTGR_Rankine_Cycle_Transient_TCV_Control_ValveDegradation_type6_Sec_Final
    HTGR_Rankine_Cycle_Transient_TCV_Control_ValveTurbineDegradation_type6_Sec_Final(
     redeclare
      NHES.Systems.BalanceOfPlant.Turbine.ControlSystems.CS_Rankine_Xe100_Based_Secondary_TransientControl_3staged_Turbine_PressControl_TCVcontrol_CompDegradation_type5_Sec
      CS(componentDegradation(Strategy_Change_Time=Strategy_Change_Time)),
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
  hTGR_PebbleBed_Primary_Loop.input_steam_pressure =
    HTGR_Rankine_Cycle_Transient_TCV_Control_ValveTurbineDegradation_type6_Sec_Final.sensor_p.p;
  Thermal_Power_Norm    = hTGR_PebbleBed_Primary_Loop.Thermal_Power.y/2.26177E8;
  Electrical_Power_Norm = HTGR_Rankine_Cycle_Transient_TCV_Control_ValveTurbineDegradation_type6_Sec_Final.generator.power;
  connect(sinkElec.port,
    HTGR_Rankine_Cycle_Transient_TCV_Control_ValveTurbineDegradation_type6_Sec_Final.port_e)
    annotation (Line(points={{84,14},{62,14}}, color={255,0,0}));
  connect(hTGR_PebbleBed_Primary_Loop.port_b, stateSensor1.port_a) annotation (
      Line(points={{-40.87,25.21},{-40.87,26},{-30,26}}, color={0,127,255}));
  connect(stateSensor1.port_b,
    HTGR_Rankine_Cycle_Transient_TCV_Control_ValveTurbineDegradation_type6_Sec_Final.port_a)
    annotation (Line(points={{-14,26},{-6,26},{-6,25.52}}, color={0,127,255}));
  connect(stateSensor2.port_a,
    HTGR_Rankine_Cycle_Transient_TCV_Control_ValveTurbineDegradation_type6_Sec_Final.port_b)
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
            fixedStepSize=0)))));
end Rankine_HTGR_threeStageTurbine_Transient_Degradation_Sec;
