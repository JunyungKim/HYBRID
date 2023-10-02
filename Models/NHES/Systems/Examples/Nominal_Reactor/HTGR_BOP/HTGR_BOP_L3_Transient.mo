within NHES.Systems.Examples.Nominal_Reactor.HTGR_BOP;
model HTGR_BOP_L3_Transient
  extends Modelica.Icons.Example;

  Real Thermal_Power_Norm;
  BalanceOfPlant.RankineCycle.Models.HTGR_RankineCycles.SteamTurbine_L3_HTGR
    BOP annotation (Placement(transformation(extent={{-6,-10},{62,38}})));
  TRANSFORM.Electrical.Sources.FrequencySource
                                     sinkElec(f=60)
    annotation (Placement(transformation(extent={{98,6},{84,22}})));
  PrimaryHeatSystem.HTGR.RankineCycle.Models.PebbleBed_PrimaryLoop_STHX hTGR_PebbleBed_Primary_Loop(
      redeclare
      NHES.Systems.PrimaryHeatSystem.HTGR.RankineCycle.ControlSystems.CS_Rankine_Primary_SS
      CS) annotation (Placement(transformation(extent={{-98,-18},{-40,40}})));
  Fluid.Sensors.stateSensor stateSensor1(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-30,16},{-14,36}})));
  Fluid.Sensors.stateSensor stateSensor2(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-14,-10},{-30,10}})));
  Fluid.Sensors.stateDisplay stateDisplay2
    annotation (Placement(transformation(extent={{-42,34},{-2,64}})));
  Fluid.Sensors.stateDisplay stateDisplay1
    annotation (Placement(transformation(extent={{-42,-10},{-2,-40}})));

equation
  hTGR_PebbleBed_Primary_Loop.input_steam_pressure =BOP.sensor_p.p;
  Thermal_Power_Norm = hTGR_PebbleBed_Primary_Loop.Thermal_Power.y/2.26177E8;
  connect(sinkElec.port, BOP.port_e)
    annotation (Line(points={{84,14},{62,14}}, color={255,0,0}));
  connect(hTGR_PebbleBed_Primary_Loop.port_b, stateSensor1.port_a) annotation (
      Line(points={{-40.87,25.21},{-40.87,26},{-30,26}}, color={0,127,255}));
  connect(stateSensor1.port_b, BOP.port_a)
    annotation (Line(points={{-14,26},{-6,26},{-6,25.52}}, color={0,127,255}));
  connect(stateSensor2.port_a, BOP.port_b)
    annotation (Line(points={{-14,0},{-6,0.08}}, color={0,127,255}));
  connect(hTGR_PebbleBed_Primary_Loop.port_a, stateSensor2.port_b) annotation (
      Line(points={{-40.87,1.43},{-40.87,0},{-30,0}}, color={0,127,255}));
  connect(stateSensor1.statePort, stateDisplay2.statePort) annotation (Line(
        points={{-21.96,26.05},{-22,26.05},{-22,45.1}}, color={0,0,0}));
  connect(stateSensor2.statePort, stateDisplay1.statePort)
    annotation (Line(points={{-22.04,0.05},{-22,-21.1}}, color={0,0,0}));
  annotation (experiment(
      StopTime=50000,
      Interval=1000,
      __Dymola_Algorithm="Esdirk45a"), Documentation(info="<html>
<p><b><span style=\"font-size: 18pt;\">Example Name</span></b></p>
<p><span style=\"font-size: 12pt;\">HTGR-3_Section_BOP (Rankine cycle with feedwater heating internal to the system) for Transient</span></p>
<p><br><b><span style=\"font-size: 18pt;\">Design Purpose of Exampe</span></b></p>
<p><span style=\"font-size: 12pt;\">Transient test of HTGR(PebbleBed)_ThreeSectionBOP(Rankine). The simulation should experience transient where external electricity demand is oscilating and control valves are opening and closing corresponding to the required power demand. </span></p>
<p><span style=\"font-size: 12pt;\">This example is developed in order to compare results with HTGR-1_ Section_BOP (Rankine cycle with feedwater heating internal to the system). Details of comparison can be found in [1]. </span></p>
<p><br><b><span style=\"font-size: 18pt;\">What Users Can Do </span></b></p>
<p><span style=\"font-size: 12pt;\">Users of this example model can test transient initiated from external electricity demand (i.e., time for demand increase; time for demand decrease; time for maximum electricity demand; one-cycle time period for electricity demand; and maximum anout of electricity demanded). See <i>Parameters</i> in the <i><span style=\"font-family: (Default);\">BOP</span></i> model.</p>
<p><br><b><span style=\"font-size: 18pt;\">Reference </span></b></p>
<p><span style=\"font-size: 12pt;\">[1] Status Report on Thermal Extraction Modeling in HYBRID (INL/RPT-23-03062)</span></p>
<p><br><b><span style=\"font-size: 18pt;\">Contact Deatils </span></b></p>
<p><span style=\"font-size: 12pt;\">This model was designed by Junyung Kim and Daniel Mikkelson. </span></p>
<p><span style=\"font-size: 12pt;\">All initial questions should be directed to Daniel Mikkelson (Daniel.Mikkelson@inl.gov). </span></p>
</html>"),
    __Dymola_Commands(executeCall=Design.Experimentation.sweepParameter(
          Design.Internal.Records.ModelDependencySetup(
          Model=
            "NHES.Systems.PrimaryHeatSystem.HTGR.HTGR_Rankine.Examples.Rankine_HTGR_Test_AR",
          dependencyParameters={Design.Internal.Records.DependencyParameter(
            name="hTGR_PebbleBed_Primary_Loop.CS.const11.k", values=linspace(
            1e-05,
            1e-06,
            6))},
          VariablesToPlot={Design.Internal.Records.VariableToPlot(name=
            "hTGR_PebbleBed_Primary_Loop.CS.CR.y"),
            Design.Internal.Records.VariableToPlot(name=
            "hTGR_PebbleBed_Primary_Loop.core.Q_total.y"),
            Design.Internal.Records.VariableToPlot(name=
            "hTGR_PebbleBed_Primary_Loop.core.fuelModel[2].T_Fuel")},
          integrator=Design.Internal.Records.Integrator(
            startTime=0,
            stopTime=1004200,
            numberOfIntervals=0,
            outputInterval=10,
            method="Esdirk45a",
            tolerance=0.0001,
            fixedStepSize=0)))),
    __Dymola_experimentSetupOutput(events=false));
end HTGR_BOP_L3_Transient;
