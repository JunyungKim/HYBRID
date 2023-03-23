within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine;
model eta_Degradation_LPT1 "Degrading efficiency_LPT1"
  extends TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.Partial_eta;
  parameter Real lambda_LPT1;

equation
  eta = eta_nominal * lambda_LPT1 *exp(-lambda_LPT1*time)*(1/lambda_LPT1);
    annotation (Dialog,
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end eta_Degradation_LPT1;
