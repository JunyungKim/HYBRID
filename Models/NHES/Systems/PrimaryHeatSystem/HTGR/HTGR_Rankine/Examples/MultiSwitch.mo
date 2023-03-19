within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples;
block MultiSwitch "Switch between two Real signals"
  extends Modelica.Blocks.Icons.PartialBooleanBlock;

  Modelica.Blocks.Interfaces.RealOutput y "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

  Modelica.Blocks.Sources.Constant p1(k=25.0e6)
    annotation (Placement(transformation(extent={{-78,66},{-58,86}})));
  Modelica.Blocks.Sources.Constant p2(k=27.5e6)
    annotation (Placement(transformation(extent={{-42,66},{-22,86}})));
  Modelica.Blocks.Sources.Constant p3(k=30.0e6)
    annotation (Placement(transformation(extent={{-10,66},{10,86}})));
  Modelica.Blocks.Sources.Constant p4(k=32.5e6)
    annotation (Placement(transformation(extent={{22,66},{42,86}})));
  Modelica.Blocks.Sources.Constant p5(k=35.0e6)
    annotation (Placement(transformation(extent={{56,66},{76,86}})));
  Modelica.Blocks.Sources.Constant p6(k=37.5e6)
    annotation (Placement(transformation(extent={{-78,32},{-58,52}})));
  Modelica.Blocks.Sources.Constant p7(k=40.0e6)
    annotation (Placement(transformation(extent={{-42,32},{-22,52}})));
  Modelica.Blocks.Sources.Constant p8(k=42.5e6)
    annotation (Placement(transformation(extent={{-10,32},{10,52}})));
  Modelica.Blocks.Sources.Constant p9(k=45.0e6)
    annotation (Placement(transformation(extent={{22,32},{42,52}})));
  Modelica.Blocks.Sources.Constant p10(k=47.5e6)
    annotation (Placement(transformation(extent={{56,32},{76,52}})));
  Modelica.Blocks.Sources.Constant p11(k=50.0e6)
    annotation (Placement(transformation(extent={{-78,0},{-58,20}})));
  parameter Real indicator = 1;
equation
  y =  if indicator == 1  then p1.k
  else if indicator == 2  then p2.y
  else if indicator == 3  then p3.y
  else if indicator == 4  then p4.y
  else if indicator == 5  then p5.y
  else if indicator == 6  then p6.y
  else if indicator == 7  then p7.y
  else if indicator == 8  then p8.y
  else if indicator == 9  then p9.y
  else if indicator == 10 then p10.y
  else if indicator == 11 then p11.y else p11.y;
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
end MultiSwitch;
