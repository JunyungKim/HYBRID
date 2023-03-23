within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine;
model eta_Degradation_LPT2 "Degrading efficiency_LPT2"
  extends TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.Partial_eta;
  parameter Real lambda_LPT2;

equation
  eta = eta_nominal * lambda_LPT2 *exp(-lambda_LPT2*time)*(1/lambda_LPT2);
    annotation (Dialog,
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end eta_Degradation_LPT2;
