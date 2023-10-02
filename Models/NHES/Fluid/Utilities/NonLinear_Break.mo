within NHES.Fluid.Utilities;
model NonLinear_Break "Oneway non linear break for fuild systems"

  TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  TRANSFORM.Fluid.Sensors.Pressure sensor_p(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{76,0},{56,20}})));
  TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{-70,10},{-50,-10}})));
  TRANSFORM.Fluid.Sensors.SpecificEnthalpy sensor_h(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{-90,0},{-70,-20}})));
  TRANSFORM.Fluid.BoundaryConditions.Boundary_ph boundary(
    redeclare package Medium = Medium,
    use_p_in=true,
    nPorts=1)
    annotation (Placement(transformation(extent={{-20,-10},{-40,10}})));
  TRANSFORM.Fluid.BoundaryConditions.MassFlowSource_h boundary1(
    redeclare package Medium = Medium,
    use_m_flow_in=true,
    use_h_in=true,
    nPorts=1) annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
    annotation (choicesAllMatching=true);
  Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=Modelica.Constants.inf)
    annotation (Placement(transformation(extent={{-40,-70},{-20,-50}})));
  Modelica.Blocks.Nonlinear.Limiter limiter2(uMax=Modelica.Constants.inf)
    annotation (Placement(transformation(extent={{40,18},{20,38}})));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax=Modelica.Constants.inf)
    annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
equation
  connect(boundary1.ports[1], sensor_p.port)
    annotation (Line(points={{40,0},{66,0}}, color={0,127,255}));
  connect(sensor_p.port, port_b)
    annotation (Line(points={{66,0},{100,0}}, color={0,127,255}));
  connect(port_a, sensor_h.port)
    annotation (Line(points={{-100,0},{-80,0}}, color={0,127,255}));
  connect(sensor_h.port, sensor_m_flow.port_a)
    annotation (Line(points={{-80,0},{-70,0}}, color={0,127,255}));
  connect(sensor_m_flow.port_b, boundary.ports[1])
    annotation (Line(points={{-50,0},{-40,0}}, color={0,127,255}));
  connect(limiter1.y, boundary1.h_in) annotation (Line(points={{-19,-60},{8,-60},
          {8,4},{18,4}}, color={0,0,127}));
  connect(limiter2.y, boundary.p_in)
    annotation (Line(points={{19,28},{-18,28},{-18,8}}, color={0,0,127}));
  connect(limiter2.u, sensor_p.p) annotation (Line(points={{42,28},{50,28},{50,
          10},{60,10}}, color={0,0,127}));
  connect(sensor_h.h_out, limiter1.u)
    annotation (Line(points={{-74,-10},{-74,-60},{-42,-60}}, color={0,0,127}));
  connect(limiter.y, boundary1.m_flow_in) annotation (Line(points={{-19,-30},{
          -20,-30},{-20,-14},{6,-14},{6,8},{20,8}},
                                               color={0,0,127}));
  connect(sensor_m_flow.m_flow, limiter.u) annotation (Line(points={{-60,-3.6},
          {-60,-30},{-42,-30}},                  color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Polygon(
          points={{20,-44},{60,-59},{20,-74},{20,-44}},
          lineColor={0,128,255},
          fillColor={0,128,255},
          fillPattern=FillPattern.Solid,
          visible=showDesignFlowDirection),
        Polygon(
          points={{20,-49},{50,-59},{20,-69},{20,-49}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          visible=allowFlowReversal),
        Line(
          points={{55,-59},{-60,-59}},
          color={0,128,255},
          visible=showDesignFlowDirection),
        Rectangle(
          extent={{-100,2},{102,-4}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-14,-20},{8,20}},
          color={28,108,200},
          thickness=1),
        Line(
          points={{-14,-22},{8,18}},
          color={28,108,200},
          thickness=1),
        Line(
          points={{-12,-20},{10,20}},
          color={28,108,200},
          thickness=1),
        Line(
          points={{-12,-22},{10,18}},
          color={28,108,200},
          thickness=1),
        Line(
          points={{-12,-24},{10,16}},
          color={28,108,200},
          thickness=1),
        Line(
          points={{-10,-22},{12,18}},
          color={28,108,200},
          thickness=1),
        Ellipse(
          extent={{-3.5,2.5},{3.5,-2.5}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          origin={9.5,18.5},
          rotation=90),
        Ellipse(
          extent={{-3,2},{3,-2}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          origin={-12,-21},
          rotation=90)}),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Non Linear Break, can be used to breakup large sets of non linear equations for fuilds.  </p><p><br><br><br><br><br>Model developed at INL by Logan Williams <span style=\"font-family: inherit;\"><a href=\"mailto:logan.williams@inl.gov\">logan.williams@inl.gov</a></span></p>
<p>Documented September 2023</p>
</html>"));
end NonLinear_Break;
