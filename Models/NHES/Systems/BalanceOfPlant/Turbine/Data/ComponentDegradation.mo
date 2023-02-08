within NHES.Systems.BalanceOfPlant.Turbine.Data;
record ComponentDegradation

  extends TRANSFORM.Icons.Record;

  // Simulation start time after Initialization
  parameter SI.Time start_after_initialization = 2e+5 "Operational Strategy Change Time";

  // Operation Strategy Change
  parameter SI.Time Strategy_Change_Time = 5e+5 "Operational Strategy Change Time";

  // TCV Degradation
  parameter Real TCV_random_coeff =     8e+12 annotation (Evaluate=true, Dialog(group="TCV Degradation"));

  // LPT BV1 Degradation
  parameter Real LPTBV1_random_coeff =     8e+12 annotation (Evaluate=true, Dialog(group="LPT BV1 Degradation"));

  // LPT BV2 Degradation
  parameter Real LPTBV2_random_coeff =     8e+12 annotation (Evaluate=true, Dialog(group="LPT BV2 Degradation"));

  // HPT Degradation
  parameter Real HPT_start_coeff =     1.0 annotation (Evaluate=true, Dialog(group="HPT_conditions"));
  parameter Real HPT_lambda =    0.0000000001 annotation (Evaluate=true, Dialog(group="HPT_conditions"));
  parameter Real HPT_coef =    1/HPT_lambda annotation (Evaluate=true, Dialog(group="HPT_conditions"));

  // LPT1 Degradation
  parameter Real LPT1_start_eff =     1.0 annotation (Evaluate=true, Dialog(group="LPT1_conditions"));
  parameter Real LPT1_lambda =    0.0000000001 annotation (Evaluate=true, Dialog(group="LPT1_conditions"));
  parameter Real LPT1_coef =    1/LPT1_lambda annotation (Evaluate=true, Dialog(group="LPT1_conditions"));

  // LPT2 Degradation
  parameter Real LPT2_start_eff =     1.0 annotation (Evaluate=true, Dialog(group="LPT2_conditions"));
  parameter Real LPT2_lambda =    0.0000000001 annotation (Evaluate=true, Dialog(group="LPT2_conditions"));
  parameter Real LPT2_coef =    1/LPT2_lambda annotation (Evaluate=true, Dialog(group="LPT2_conditions"));


  // Feedwater Pump Degradation
  parameter Real pump_random_coeff1 =    1.0 annotation (Evaluate=true, Dialog(group="CirculationPump_conditions"));
  parameter Real pump_random_coeff2 =    3e12 "3e12 for 1000000 sec. simulation" annotation (Evaluate=true, Dialog(group="CirculationPump_conditions"));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                                                Text(
          lineColor={0,0,0},
          extent={{-100,-90},{100,-70}},
          textString="BOP_3stage")}),                            Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ComponentDegradation;
