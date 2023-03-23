within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine;
block MultiSwitch_6 "Switch between two Real signals"
  extends Modelica.Blocks.Icons.PartialBooleanBlock;

  Modelica.Blocks.Interfaces.RealOutput y "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

  parameter Real indicator = 1;
  Modelica.Blocks.Sources.Constant p1(k=24.0e6)
    annotation (Placement(transformation(extent={{-64,52},{-44,72}})));
  Modelica.Blocks.Sources.Constant p2(k=28.0e6)
    annotation (Placement(transformation(extent={{-28,52},{-8,72}})));
  Modelica.Blocks.Sources.Constant p3(k=32.0e6)
    annotation (Placement(transformation(extent={{4,52},{24,72}})));
  Modelica.Blocks.Sources.Constant p4(k=36.0e6)
    annotation (Placement(transformation(extent={{36,52},{56,72}})));
  Modelica.Blocks.Sources.Constant p5(k=40.0e6)
    annotation (Placement(transformation(extent={{-64,16},{-44,36}})));
  Modelica.Blocks.Sources.Constant p6(k=44.0e6)
    annotation (Placement(transformation(extent={{-28,16},{-8,36}})));
equation
  y =  if indicator == 1  then p1.k
  else if indicator == 2  then p2.y
  else if indicator == 3  then p3.y
  else if indicator == 4  then p4.y
  else if indicator == 5  then p5.y
  else if indicator == 6  then p6.y else p6.y;
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
end MultiSwitch_6;
