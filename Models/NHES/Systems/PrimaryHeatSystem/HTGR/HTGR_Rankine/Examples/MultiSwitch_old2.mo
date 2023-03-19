within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples;
block MultiSwitch_old2 "Switch between two Real signals"
  extends Modelica.Blocks.Icons.PartialBooleanBlock;

  Modelica.Blocks.Interfaces.RealOutput y "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

  parameter Modelica.Blocks.Sources.RealExpression indicator(y=1)
    annotation (Placement(transformation(extent={{-90,-110},{-70,-90}})));

  Modelica.Blocks.Sources.RealExpression p1(y=25.0e6)
    annotation (Placement(transformation(extent={{-90,78},{-70,98}})));
  Modelica.Blocks.Sources.RealExpression p2(y=27.5e6)
    annotation (Placement(transformation(extent={{-90,62},{-70,82}})));
  Modelica.Blocks.Sources.RealExpression p3(y=30.0e6)
    annotation (Placement(transformation(extent={{-90,46},{-70,66}})));
  Modelica.Blocks.Sources.RealExpression p4(y=32.5e6)
    annotation (Placement(transformation(extent={{-90,30},{-70,50}})));
  Modelica.Blocks.Sources.RealExpression p5(y=37.5e6)
    annotation (Placement(transformation(extent={{-90,-4},{-70,16}})));
  Modelica.Blocks.Sources.RealExpression p6(y=35.0e6)
    annotation (Placement(transformation(extent={{-90,14},{-70,34}})));
  Modelica.Blocks.Sources.RealExpression p7(y=40.0e6)
    annotation (Placement(transformation(extent={{-90,-22},{-70,-2}})));
  Modelica.Blocks.Sources.RealExpression p8(y=42.5e6)
    annotation (Placement(transformation(extent={{-90,-40},{-70,-20}})));
  Modelica.Blocks.Sources.RealExpression p9(y=45.0e6)
    annotation (Placement(transformation(extent={{-90,-58},{-70,-38}})));
  Modelica.Blocks.Sources.RealExpression p10(y=47.5e6)
    annotation (Placement(transformation(extent={{-90,-76},{-70,-56}})));
  Modelica.Blocks.Sources.RealExpression p11(y=50.0e6)
    annotation (Placement(transformation(extent={{-90,-92},{-70,-72}})));
equation
  // y = if u2 > 0 then u1 else u3;
  y =  if indicator.y == 1  then p1.y
  else if indicator.y == 2  then p2.y
  else if indicator.y == 3  then p3.y
  else if indicator.y == 4 then p4.y
  else if indicator.y == 5 then p5.y
  else if indicator.y == 6 then p6.y
  else if indicator.y == 7 then p7.y
  else if indicator.y == 8 then p8.y
  else if indicator.y == 9 then p9.y
  else if indicator.y == 10 then p10.y
  else if indicator.y == 11 then p11.y else p11.y;
  annotation (
    defaultComponentName="switch1",
    Documentation(info="<html>
<p>The Logical.Switch switches, depending on the
logical connector u2 (the middle connector)
between the two possible input signals
u1 (upper connector) and u3 (lower connector).</p>
<p>If u2 is <strong>true</strong>, the output signal y is set equal to
u1, else it is set equal to u3.</p>
</html>"),
    Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}})));
end MultiSwitch_old2;
