within NHES.Electrolysis.Electrical;
model SwitchYard_LooselyCoupled
  import      Modelica.Units.SI;

  SI.Power totalLoad_elecHeaters;
  SI.Power powerConsumption_SOEC(start=45.53135e6, min = 16.596350e6, max=45.53135e6*1.05);

  Interfaces.ElectricalPowerPort_a totalElecPower annotation (Placement(
        transformation(extent={{70,70},{90,90}}), iconTransformation(extent={{
            70,70},{90,90}})));
  Interfaces.ElectricalPowerPort_b load_SOEC
    annotation (Placement(transformation(extent={{70,-90},{90,-70}})));
  Interfaces.ElectricalPowerPort_b load_catElecHeater2
    annotation (Placement(transformation(extent={{-90,70},{-70,90}})));
  Interfaces.ElectricalPowerPort_b load_anElecHeater
    annotation (Placement(transformation(extent={{-90,-90},{-70,-70}})));
  Sources.PowerSink totalElec_sink
    annotation (Placement(transformation(extent={{60,70},{40,90}})));
  Sources.PrescribedPowerFlow SOECelec_source
    annotation (Placement(transformation(extent={{40,-90},{60,-70}})));
  Modelica.Blocks.Sources.RealExpression SOECelec_sourceSingal(y=
        powerConsumption_SOEC)
    annotation (Placement(transformation(extent={{10,-90},{30,-70}})));
  Sources.LoadSink catElec_sink2
    annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
  Sources.LoadSink anElec_sink
    annotation (Placement(transformation(extent={{-60,-90},{-40,-70}})));
  Interfaces.ElectricalPowerPort_b load_catElecHeater1
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
        iconTransformation(extent={{-110,-10},{-90,10}})));
  Sources.LoadSink catElec_sink1
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
equation

  totalLoad_elecHeaters =-(catElec_sink1.W + catElec_sink2.W + anElec_sink.W);
  powerConsumption_SOEC = totalElec_sink.W - totalLoad_elecHeaters;

  connect(SOECelec_source.P_flow, SOECelec_sourceSingal.y)
  annotation (Line(points={{42,-80},{31,-80}}, color={0,0,127}));

  connect(load_catElecHeater2, catElec_sink2.port_a) annotation (Line(
      points={{-80,80},{-70,80},{-60,80}},
      color={255,0,0},
      thickness=0.5));
  connect(load_anElecHeater, anElec_sink.port_a) annotation (Line(
      points={{-80,-80},{-70,-80},{-60,-80}},
      color={255,0,0},
      thickness=0.5));
  connect(load_SOEC, SOECelec_source.port_b) annotation (Line(
      points={{80,-80},{70,-80},{60,-80}},
      color={255,0,0},
      thickness=0.5));
  connect(totalElecPower, totalElec_sink.port_a) annotation (Line(
      points={{80,80},{70,80},{60,80}},
      color={255,0,0},
      thickness=0.5));
  connect(load_catElecHeater1, catElec_sink1.port_a) annotation (Line(
      points={{-100,0},{-90,0},{-80,0}},
      color={255,0,0},
      thickness=0.5));
annotation (defaultComponentName="switchYard",Icon(coordinateSystem(preserveAspectRatio=false,
        extent={{-100,-100},{100,100}}), graphics={
      Polygon(
        points={{-100,-60},{-60,-100},{58,-100},{100,-60},{100,60},{60,100},{-60,
            100},{-100,60},{-100,-60}},
        lineColor={175,175,175},
        fillColor={255,213,170},
        fillPattern=FillPattern.CrossDiag),
      Line(
        points={{-80,52},{-40,0},{-2,52},{-40,0},{-40,-52}},
        color={255,0,0},
        thickness=0.5),
      Line(
        points={{0,40},{0,-40},{80,0},{0,40}},
        color={255,0,0},
        thickness=0.5),
              Text(
        extent={{-150,-109},{150,-149}},
        lineColor={0,0,255},
        textString="%name")}), Diagram(coordinateSystem(preserveAspectRatio=false,
        extent={{-100,-100},{100,100}})));
end SwitchYard_LooselyCoupled;
