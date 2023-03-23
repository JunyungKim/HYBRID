within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine;
model eta_Degradation_HPT "Degrading efficiency_HPT"
  extends TRANSFORM.Fluid.Machines.BaseClasses.WetSteamEfficiency.Partial_eta;
  parameter Real lambda_HPT;

equation
  eta = eta_nominal * lambda_HPT *exp(-lambda_HPT*time)*(1/lambda_HPT);

    annotation (Dialog,
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end eta_Degradation_HPT;
