within NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples;
block MultiSwitch_old "Switch between two Real signals"
  extends Modelica.Blocks.Icons.PartialBooleanBlock;
  Modelica.Blocks.Interfaces.RealInput u1
    "Connector of first Real input signal"
    annotation (Placement(transformation(extent={{-116,84},{-100,100}}),
        iconTransformation(extent={{-116,84},{-100,100}})));
  Modelica.Blocks.Interfaces.RealInput u2
    "Connector of Boolean input signal"
    annotation (Placement(transformation(extent={{-116,66},{-100,82}}),
        iconTransformation(extent={{-116,66},{-100,82}})));
  Modelica.Blocks.Interfaces.RealInput u3
    "Connector of second Real input signal"
    annotation (Placement(transformation(extent={{-116,50},{-100,66}}),
        iconTransformation(extent={{-116,50},{-100,66}})));

  Modelica.Blocks.Interfaces.RealOutput y "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

  Modelica.Blocks.Sources.RealExpression indicator(y=1)
    annotation (Placement(transformation(extent={{-98,102},{-78,122}})));

  Modelica.Blocks.Interfaces.RealInput u4
    "Connector of second Real input signal"
    annotation (Placement(transformation(extent={{-116,34},{-100,50}}),
        iconTransformation(extent={{-116,34},{-100,50}})));
  Modelica.Blocks.Interfaces.RealInput u5
    "Connector of second Real input signal"
    annotation (Placement(transformation(extent={{-116,18},{-100,34}}),
        iconTransformation(extent={{-116,18},{-100,34}})));
  Modelica.Blocks.Interfaces.RealInput u6
    "Connector of second Real input signal"
    annotation (Placement(transformation(extent={{-116,-8},{-100,8}}),
        iconTransformation(extent={{-116,-8},{-100,8}})));
  Modelica.Blocks.Interfaces.RealInput u7
    "Connector of second Real input signal"
    annotation (Placement(transformation(extent={{-116,-32},{-100,-16}}),
        iconTransformation(extent={{-116,-32},{-100,-16}})));
  Modelica.Blocks.Interfaces.RealInput u8
    "Connector of second Real input signal"
    annotation (Placement(transformation(extent={{-116,-48},{-100,-32}}),
        iconTransformation(extent={{-116,-48},{-100,-32}})));
  Modelica.Blocks.Interfaces.RealInput u9
    "Connector of second Real input signal"
    annotation (Placement(transformation(extent={{-116,-64},{-100,-48}}),
        iconTransformation(extent={{-116,-64},{-100,-48}})));
  Modelica.Blocks.Interfaces.RealInput u10
    "Connector of second Real input signal"
    annotation (Placement(transformation(extent={{-116,-80},{-100,-64}}),
        iconTransformation(extent={{-116,-80},{-100,-64}})));
  Modelica.Blocks.Interfaces.RealInput u11
    "Connector of second Real input signal"
    annotation (Placement(transformation(extent={{-116,-80},{-100,-64}}),
        iconTransformation(extent={{-116,-96},{-100,-80}})));
equation
  // y = if u2 > 0 then u1 else u3;
  y =  if indicator.y == 1 then u1
  else if indicator.y == 2 then u2
  else if indicator.y == 3 then u3
  else if indicator.y == 4 then u4
  else if indicator.y == 5 then u5
  else if indicator.y == 6 then u6
  else if indicator.y == 7 then u7
  else if indicator.y == 8 then u8
  else if indicator.y == 9 then u9
  else if indicator.y == 10 then u10
  else if indicator.y == 11 then u11 else u11;
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
        extent={{-100,-100},{100,100}}), graphics={
        Line(points={{12,0},{100,0}},
          color={0,0,127}),
        Line(points=DynamicSelect({{-38,80},{6,2}}, if u2 then {{-38,80},{6,2}} else {{-38,-80},{6,2}}),
          color={0,0,127},
          thickness=1),
        Ellipse(lineColor={0,0,255},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{2,-8},{18,8}})}));
end MultiSwitch_old;
