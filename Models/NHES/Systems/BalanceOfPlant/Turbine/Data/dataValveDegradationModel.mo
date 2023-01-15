within NHES.Systems.BalanceOfPlant.Turbine.Data;
record dataValveDegradationModel

  extends TRANSFORM.Icons.Record;

  // Strategy Change Time
  parameter SI.Time strategyChangeTime =  283824000 annotation (Evaluate=true, Dialog(group="Strategy Change Time: 283824000 = 9year"));

  // TCV Degradation
  parameter Integer TCV_randomSeed_failToOperate =     1234 annotation (Evaluate=true, Dialog(group="TCV Degradation"));
  parameter Integer TCV_randomSeed_failToRemainOpen =     2345 annotation (Evaluate=true, Dialog(group="TCV Degradation"));

  // LPT BV1 Degradation
  parameter Integer LPTBV1_randomSeed_failToOperate =     5678 annotation (Evaluate=true, Dialog(group="LPT BV1 Degradation"));
  parameter Integer LPTBV1_randomSeed_failToRemainOpen =     6789 annotation (Evaluate=true, Dialog(group="LPT BV1 Degradation"));

  // LPT BV2 Degradation
  parameter Integer LPTBV2_randomSeed_failToOperate =     8912 annotation (Evaluate=true, Dialog(group="LPT BV1 Degradation"));
  parameter Integer LPTBV2_randomSeed_failToRemainOpen =     9123 annotation (Evaluate=true, Dialog(group="LPT BV1 Degradation"));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                                                Text(
          lineColor={0,0,0},
          extent={{-100,-90},{100,-70}},
          textString="BOP_3stage")}),                            Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end dataValveDegradationModel;
