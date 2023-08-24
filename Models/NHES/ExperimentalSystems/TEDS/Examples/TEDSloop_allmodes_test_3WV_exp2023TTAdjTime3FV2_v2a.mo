within NHES.ExperimentalSystems.TEDS.Examples;
model TEDSloop_allmodes_test_3WV_exp2023TTAdjTime3FV2_v2a
  "Test designed to ensure the TEDS loop can operate in all modes."

  parameter Real FV_opening=0.250;

  TRANSFORM.Fluid.Pipes.GenericPipe_MultiTransferSurface Chromolox_Heater(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    p_a_start=system.p_ambient,
    T_a_start=system.T_start,
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.GenericPipe
        (nV=3, dimensions=fill(0.076, Chromolox_Heater.nV)),
    redeclare model FlowModel =
        TRANSFORM.Fluid.ClosureRelations.PressureLoss.Models.DistributedPipe_1D.SinglePhase_Developed_2Region_NumStable,
    use_HeatTransfer=true,
    redeclare model HeatTransfer =
        TRANSFORM.Fluid.ClosureRelations.HeatTransfer.Models.DistributedPipe_1D_MultiTransferSurface.Ideal)
    annotation (Placement(transformation(extent={{-48,38},{-32,54}})));

  Modelica.Fluid.Pipes.DynamicPipe pipe2(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    length=1,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=600, m_flow_nominal=0.689))
    annotation (Placement(transformation(extent={{46,68},{62,84}})));
  Modelica.Fluid.Pipes.DynamicPipe pipe4(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    length=1,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=600, m_flow_nominal=0.689))
                    annotation (Placement(transformation(
        extent={{-9,-9},{9,9}},
        rotation=90,
        origin={-95,-47})));
  Modelica.Fluid.Pipes.DynamicPipe pipe7(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    length=0.1,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=600, m_flow_nominal=0.84))
                    annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=270,
        origin={228,-18})));
  TRANSFORM.Fluid.Volumes.ExpansionTank tank1(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    A=1,
    V0=0.1,
    p_surface=system.p_ambient,
    p_start=system.p_start,
    level_start=0,
    h_start=Chromolox_Heater.h_b_start)
    annotation (Placement(transformation(extent={{-6,72},{10,88}})));
  inner TRANSFORM.Fluid.System
                        system(
    p_ambient=18000,
    T_ambient=298.15,
    m_flow_start=0.84)
    annotation (Placement(transformation(extent={{220,120},{240,140}})));
  Data.Data_TEDS data(T_hot_side=523.15, T_cold_side=298.15)
    annotation (Placement(transformation(extent={{-136,118},{-116,138}})));

  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_002(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(extent={{-80,34},{-56,56}})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_003(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(extent={{-28,34},{-4,58}})));
  Modelica.Fluid.Pipes.DynamicPipe pipe3(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    length=1,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=600, m_flow_nominal=0.84))
                    annotation (Placement(transformation(
        extent={{6,6},{-6,-6}},
        rotation=0,
        origin={170,-142})));
  TRANSFORM.Fluid.Sensors.MassFlowRate m_thot(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={84,32})));
  TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow2(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision=
       3) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={228,48})));
  TRANSFORM.Fluid.Sensors.MassFlowRate FM_002(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{13,10},{-13,-10}},
        rotation=90,
        origin={123,-186})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_049(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    m_flow_start=1e-2,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={84,66})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_050(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={154,66})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_051(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={80,-116})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_052(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={154,-128})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_006(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    m_flow_start=0.41,
    dp_nominal=3000,
    m_flow_nominal=0.840,
    dp(start=100, fixed=true))
                          annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={132,76})));
  TRANSFORM.Fluid.Sensors.MassFlowRate FM_201(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-12,-10},{12,10}},
        rotation=0,
        origin={52,-122})));
  TRANSFORM.Fluid.Sensors.MassFlowRate FM_202(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-11,-10},{11,10}},
        rotation=-90,
        origin={154,-101})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_004(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    m_flow_start=0.41,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={-90,-146})));
  TRANSFORM.Fluid.Sensors.MassFlowRate FM_003(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{9,-8},{-9,8}},
        rotation=270,
        origin={-103,-128})));
  Control_Systems.Control_System_Therminol_4_element_all_modes_ExpTest
    control_System_Therminol_4_element_all_modes_ExpTest(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    Heater_max=225e3,
    T_hot_design=523.15,
    T_cold_design=303.15,
    Heater_Demand(table=[0,0.05; 3500,0.05; 3600,1; 4800,1; 7200,1; 9000,1;
          9600,1; 10800,1; 12600,1; 12700,0.75; 17900,0.75; 18000,0.005; 23400,
          0.005; 25000,0.005]),
    BOP_relative_demand(table=[0,10; 3500,10; 3600,1; 4800,1; 7200,1; 9600,1;
          12500,1; 12600,150; 14400,150; 17900,150; 18000,150; 23300,150; 23400,
          1; 25000,1]))
    annotation (Placement(transformation(extent={{20,118},{42,140}})));
  BaseClasses.SignalSubBus_ActuatorInput actuatorSubBus
    annotation (Placement(transformation(extent={{-42,94},{-20,118}})));
  BaseClasses.SignalSubBus_SensorOutput sensorSubBus
    annotation (Placement(transformation(extent={{-10,94},{12,118}})));
  TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow3(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision=
       3) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={154,40})));
  ThermoclineModels.Thermocline_Insulation thermocline_Insulation(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    redeclare package InsulationMaterial = NHES.Media.Solids.FoamGlass,
    geometry(
      Radius_Tank=0.438,
      Porosity=0.5,
      dr=0.00317,
      Insulation_thickness=3*0.051,
      Wall_Thickness=0.019,
      Height_Tank=4.435),
    T_Init=303.15)
    annotation (Placement(transformation(extent={{102,-50},{134,-6}})));
  Modelica.Fluid.Sources.MassFlowSource_T Chiller_Mass_Flow(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.EthyleneGlycol.LinearEthyleneGlycol_50_Water,
    use_m_flow_in=true,
    m_flow=12.6,
    T=280.15,
    nPorts=1)
    annotation (Placement(transformation(extent={{30,-244},{50,-224}})));

  Modelica.Fluid.Sources.Boundary_pT boundary1(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.EthyleneGlycol.LinearEthyleneGlycol_50_Water,
    p=300000,
    T=291.15,
    nPorts=1)
    annotation (Placement(transformation(extent={{210,-220},{190,-240}})));

  TRANSFORM.HeatExchangers.GenericDistributed_HX Glycol_HX(
    p_b_start_shell=system.p_ambient,
    T_a_start_shell=299.15,
    T_b_start_shell=298.15,
    p_b_start_tube=boundary1.p,
    counterCurrent=true,
    m_flow_a_start_tube=Chiller_Mass_Flow.m_flow,
    m_flow_a_start_shell=12.6,
    redeclare package Medium_tube =
        TRANSFORM.Media.Fluids.EthyleneGlycol.LinearEthyleneGlycol_50_Water,
    redeclare package Medium_shell =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    redeclare package Material_tubeWall = TRANSFORM.Media.Solids.SS316,
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.HeatExchanger.ShellAndTubeHX
        (
        D_o_shell=0.44,
        nV=10,
        nTubes=1,
        nR=3,
        length_shell=0.37*1,
        dimension_tube=0.05 - 2*Glycol_HX.geometry.th_wall,
        length_tube=22.0*1,
        th_wall=0.0015),
    p_a_start_tube=boundary1.p + 100,
    T_a_start_tube=Chiller_Mass_Flow.T,
    T_b_start_tube=boundary1.T,
    p_a_start_shell=system.p_ambient + 100,
    redeclare model HeatTransfer_tube =
        TRANSFORM.Fluid.ClosureRelations.HeatTransfer.Models.DistributedPipe_1D_MultiTransferSurface.Nus_DittusBoelter_Simple
        (CF=1.0),
    redeclare model HeatTransfer_shell =
        TRANSFORM.Fluid.ClosureRelations.HeatTransfer.Models.DistributedPipe_1D_MultiTransferSurface.Nus_DittusBoelter_Simple
        (CF=2.0))
    annotation (Placement(transformation(extent={{69,-244},{100,-214}})));

  TRANSFORM.Fluid.Sensors.TemperatureTwoPort Ethylene_glycol_exit_temperature(
      redeclare package Medium =
        TRANSFORM.Media.Fluids.EthyleneGlycol.LinearEthyleneGlycol_50_Water,
      precision=3)
    annotation (Placement(transformation(extent={{140,-242},{170,-218}})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_006(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(extent={{28,-136},{52,-162}})));
  Modelica.Blocks.Sources.Constant const(k=
        control_System_Therminol_4_element_all_modes_ExpTest.T_hot_design)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={-156,46})));
  TRANSFORM.Controls.LimPID Chromolox_Heater_Control(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=10,
    Ti=0.03,
    k_s=1,
    k_m=1,
    yMax=225e3,
    yMin=0,
    initType=Modelica.Blocks.Types.Init.InitialOutput,
    y_start=200e3)
    annotation (Placement(transformation(extent={{-122,52},{-110,40}})));
  Modelica.Blocks.Sources.RealExpression realExpression[Chromolox_Heater.geometry.nV](
     y=fill(Chromolox_Heater_Control.y/Chromolox_Heater.geometry.nV,
        Chromolox_Heater.geometry.nV))
    annotation (Placement(transformation(extent={{-94,54},{-74,74}})));
  TRANSFORM.HeatAndMassTransfer.BoundaryConditions.Heat.HeatFlow_multi
    boundary3(nPorts=Chromolox_Heater.geometry.nV, use_port=true)
    annotation (Placement(transformation(extent={{-68,54},{-48,74}})));
  Modelica.Blocks.Math.Sum chromoloxHeater_Power(nin=Chromolox_Heater.geometry.nV)
    annotation (Placement(transformation(extent={{-72,86},{-60,98}})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort T_ch_o(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-12,13},{12,-13}},
        rotation=90,
        origin={154,-63})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_201(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-12,13},{12,-13}},
        rotation=-90,
        origin={84,7})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort T_discharge_outlet(redeclare
      package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision=
       3) annotation (Placement(transformation(
        extent={{-12,13},{12,-13}},
        rotation=90,
        origin={154,9})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_202(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-12,-13},{12,13}},
        rotation=90,
        origin={80,-63})));
  TRANSFORM.Fluid.Sensors.MassFlowRate BOP_Mass_flow(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision=
       3) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={106,76})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_004(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-10,12},{10,-12}},
        rotation=90,
        origin={124,-212})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort T_chiller_before(redeclare package
      Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
      precision=3) annotation (Placement(transformation(
        extent={{-13,12},{13,-12}},
        rotation=270,
        origin={228,15})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort T_chiller_after(redeclare package
      Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
      precision=3) annotation (Placement(transformation(
        extent={{-13,13},{13,-13}},
        rotation=270,
        origin={227,-65})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_003a(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(extent={{18,64},{42,88}})));
  TRANSFORM.Fluid.Machines.Pump_PressureBooster P_001(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    use_input=true,
    p_nominal=system.p_ambient + 1e4)
    annotation (Placement(transformation(extent={{0,-138},{-16,-154}})));
  Modelica.Blocks.Sources.RealExpression Heater_BOP_Demand(y=P_001.port_a.p +
        2.0e4)
    annotation (Placement(transformation(extent={{-54,-180},{-32,-158}})));
  Modelica.Blocks.Sources.Constant const2(k=12.6)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-4,-228})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_012(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={56,-202})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_009(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{6,6},{-6,-6}},
        rotation=0,
        origin={92,-144})));
  TRANSFORM.Controls.LimPID MassFlow_Control1(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=-0.0004,
    Ti=5,
    yMax=0.999,
    yMin=0.001,
    initType=Modelica.Blocks.Types.Init.InitialOutput,
    y_start=0.5)
    annotation (Placement(transformation(extent={{-34,-202},{-22,-214}})));
  Modelica.Blocks.Sources.RealExpression BVctr(y=1 - PV_012.opening)
    annotation (Placement(transformation(extent={{74,-176},{96,-154}})));
  Modelica.Blocks.Sources.Constant const1(k=1)           annotation (Placement(
        transformation(
        extent={{4,-4},{-4,4}},
        rotation=0,
        origin={144,-150})));
  Fluid.Pipes.NonLinear_Break nonLinear_Break(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C)
    annotation (Placement(transformation(extent={{6,-8},{-6,8}},
        rotation=270,
        origin={56,-216})));
  Fluid.Pipes.NonLinear_Break nonLinear_Break2(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C)
    annotation (Placement(transformation(extent={{6,-8},{-6,8}},
        rotation=0,
        origin={16,-146})));
  Modelica.Blocks.Sources.CombiTimeTable V1(table=[0,0; 2220,0; 2400,1; 9180,1;
        9280,0.0001; 10980,0.0001; 11080,0.0001; 14640,0.0001; 14740,0.0001;
        15740,0.0001],                                         startTime=0)
    annotation (Placement(transformation(extent={{42,158},{56,172}})));
  Modelica.Blocks.Sources.CombiTimeTable V2(table=[0,1; 2220,1; 2400,FV_opening;
        9180,FV_opening; 9280,1; 10980,1; 11080,FV_opening; 14640,FV_opening; 14740,
        1; 15740,1],   startTime=0)
    annotation (Placement(transformation(extent={{92,158},{106,172}})));
  Modelica.Blocks.Sources.CombiTimeTable V3(table=[0,0.0001; 2220,0; 2400,0;
        9180,0; 9280,0; 10980,0; 11080,1; 14640,1; 14740,0; 15740,0],
                                                               startTime=0)
    annotation (Placement(transformation(extent={{134,158},{148,172}})));
  TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow1(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(extent={{-24,-154},{-42,-138}})));
  TRANSFORM.Fluid.Valves.ValveLinear ValveFl(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    m_flow_start=0.41,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{6,-6},{-6,6}},
        rotation=0,
        origin={-54,-146})));
  Modelica.Blocks.Sources.CombiTimeTable Flow(table=[0,44; 2220,44; 2400,34;
        2760,34; 2860,23; 3120,23; 3220,15; 3840,15; 3940,13; 5280,13; 5380,
        11.2; 9180,11.2; 9280,10.1; 10980,10.8; 11080,11.1; 14640,11.1; 14740,
        12; 15740,12],
      startTime=0)
    annotation (Placement(transformation(extent={{-76,-80},{-62,-66}})));
  TRANSFORM.Controls.LimPID VolFlow_Control2(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.04,
    Ti=50,
    yMax=0.99,
    yMin=0.01,
    initType=Modelica.Blocks.Types.Init.InitialOutput,
    y_start=0.5)
    annotation (Placement(transformation(extent={{-42,-80},{-30,-68}})));
  Fluid.Pipes.NonLinear_Break nonLinear_Break1(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C)
    annotation (Placement(transformation(extent={{-6,-8},{6,8}},
        rotation=0,
        origin={-16,-126})));
  Modelica.Blocks.Sources.RealExpression Heater_BOP_Demand2(y=1/
        sensor_m_flow1.Medium.density_ph(sensor_m_flow1.port_b.p,
        sensor_m_flow1.port_b.h_outflow))
    annotation (Placement(transformation(extent={{-194,-122},{-172,-100}})));
  Modelica.Blocks.Sources.RealExpression GPMconversion(y=15850.323140625002)
    annotation (Placement(transformation(extent={{-194,-100},{-172,-78}})));
  Modelica.Blocks.Math.Product product1
    annotation (Placement(transformation(extent={{-154,-116},{-144,-106}})));
  Modelica.Blocks.Math.Product FM_001
    annotation (Placement(transformation(extent={{-126,-98},{-116,-88}})));
  Modelica.Blocks.Sources.CombiTimeTable THXout(table=[0,400; 2220,400; 2400,
        400; 9180,400; 9280,138; 10980,138; 11080,138; 14640,138; 14740,138;
        15740,138], startTime=0)
    annotation (Placement(transformation(extent={{-112,-202},{-98,-188}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{-84,-218},{-64,-198}})));
  Modelica.Blocks.Sources.Constant const3(k=273.15)      annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-110,-214})));
  Modelica.Blocks.Continuous.FirstOrder firstOrder5(
    T=5,
    initType=Modelica.Blocks.Types.Init.NoInit,
    y_start=1)
    annotation (Placement(transformation(extent={{112,-122},{100,-110}})));
  Modelica.Blocks.Sources.Constant const4(k=273.15)      annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-190,78})));
  Modelica.Blocks.Sources.CombiTimeTable THeater(table=[0,300; 3000,300; 3060,
        250; 10980,250; 11080,0; 12000,0.0],
                                  startTime=0)
    annotation (Placement(transformation(extent={{-190,96},{-176,110}})));
  Modelica.Blocks.Math.Add add1
    annotation (Placement(transformation(extent={{-164,72},{-144,92}})));
  Modelica.Blocks.Sources.RealExpression Q_GHX(y=Glycol_HX.port_a_shell.m_flow*
        (FM_002.port_b.h_outflow - Glycol_HX.port_b_shell.h_outflow))
    annotation (Placement(transformation(extent={{72,-270},{94,-248}})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_005(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-10,12},{10,-12}},
        rotation=270,
        origin={56,-160})));
  Fluid.Pipes.NonLinear_Break nonLinear_Break3(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C)
    annotation (Placement(transformation(extent={{6,-8},{-6,8}},
        rotation=0,
        origin={112,-142})));
  Modelica.Fluid.Pipes.DynamicPipe p1(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    length=10,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=600, m_flow_nominal=0.689),
    T_start=298.15)
    annotation (Placement(transformation(extent={{16,-130},{32,-114}})));
  Modelica.Fluid.Pipes.DynamicPipe pipe5(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    length=2,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=600, m_flow_nominal=0.689),
    T_start=298.15)
    annotation (Placement(transformation(extent={{90,48},{106,64}})));
  Modelica.Fluid.Pipes.DynamicPipe pipe6(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    length=10,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=600, m_flow_nominal=0.689),
    T_start=298.15)
    annotation (Placement(transformation(extent={{-6,-6},{6,6}},
        rotation=90,
        origin={80,-86})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_008(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{6,6},{-6,-6}},
        rotation=90,
        origin={124,-150})));
  Modelica.Fluid.Pipes.DynamicPipe pipe1(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    length=4,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=600, m_flow_nominal=0.689),
    T_start=298.15)
    annotation (Placement(transformation(extent={{-8,-8},{8,8}},
        rotation=270,
        origin={140,24})));
  Modelica.Fluid.Pipes.DynamicPipe pipe8(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    length=2,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=600, m_flow_nominal=0.689),
    T_start=298.15)
    annotation (Placement(transformation(extent={{-6,-6},{6,6}},
        rotation=90,
        origin={134,-82})));
  Modelica.Fluid.Pipes.DynamicPipe p2(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    length=6,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=600, m_flow_nominal=0.689))
    annotation (Placement(transformation(extent={{128,-172},{144,-156}})));
equation
  connect(pipe4.port_b, TC_002.port_a)
    annotation (Line(points={{-95,-38},{-95,45},{-80,45}}, color={0,127,255}));
  connect(TC_002.port_b, Chromolox_Heater.port_a)
    annotation (Line(points={{-56,45},{-56,46},{-48,46}}, color={0,127,255}));
  connect(Chromolox_Heater.port_b, TC_003.port_a)
    annotation (Line(points={{-32,46},{-28,46}}, color={0,127,255}));
  connect(TC_003.port_b, tank1.port_a) annotation (Line(points={{-4,46},{-3.6,
          46},{-3.6,75.2}}, color={0,127,255}));
  connect(pipe2.port_b,PV_049. port_a)
    annotation (Line(points={{62,76},{84,76},{84,72}}, color={0,127,255}));
  connect(PV_050.port_a, sensor_m_flow2.port_a) annotation (Line(points={{154,72},
          {154,76},{228,76},{228,58}}, color={0,127,255}));
  connect(PV_006.port_b, sensor_m_flow2.port_a)
    annotation (Line(points={{138,76},{228,76},{228,58}}, color={0,127,255}));
  connect(PV_052.port_a, FM_202.port_b)
    annotation (Line(points={{154,-122},{154,-112}}, color={0,127,255}));
  connect(PV_004.port_a, FM_003.port_a) annotation (Line(points={{-96,-146},{-100,
          -146},{-100,-137},{-103,-137}}, color={0,127,255}));
  connect(FM_003.port_b, pipe4.port_a) annotation (Line(points={{-103,-119},{-104,
          -119},{-104,-62},{-95,-62},{-95,-56}}, color={0,127,255}));
  connect(control_System_Therminol_4_element_all_modes_ExpTest.actuatorSubBus,
    actuatorSubBus) annotation (Line(
      points={{30.7067,118.092},{30.7067,106},{-31,106}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(control_System_Therminol_4_element_all_modes_ExpTest.sensorSubBus,
    sensorSubBus) annotation (Line(
      points={{36.1333,118.092},{36.1333,106},{1,106}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(sensorSubBus.Valve_6_Opening,PV_004. opening) annotation (Line(
      points={{1,106},{14,106},{14,64},{8,64},{8,-104},{-78,-104},{-78,-136},{
          -90,-136},{-90,-141.2}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(PV_050.port_b, sensor_m_flow3.port_b)
    annotation (Line(points={{154,60},{154,50}}, color={0,127,255}));
  connect(Chiller_Mass_Flow.ports[1], Glycol_HX.port_a_tube) annotation (Line(
        points={{50,-234},{50,-229},{69,-229}},           color={0,127,255}));
  connect(Glycol_HX.port_b_tube, Ethylene_glycol_exit_temperature.port_a)
    annotation (Line(points={{100,-229},{100,-230},{140,-230}},           color=
         {0,127,255}));
  connect(Ethylene_glycol_exit_temperature.port_b, boundary1.ports[1])
    annotation (Line(points={{170,-230},{190,-230}}, color={0,127,255}));
  connect(realExpression.y, boundary3.Q_flow_ext)
    annotation (Line(points={{-73,64},{-62,64}}, color={0,0,127}));
  connect(boundary3.port, Chromolox_Heater.heatPorts[:, 1])
    annotation (Line(points={{-48,64},{-40,64},{-40,50}}, color={191,0,0}));
  connect(TC_003.T, Chromolox_Heater_Control.u_m) annotation (Line(points={{-16,
          50.32},{-16,84},{-116,84},{-116,53.2}}, color={0,0,127}));
  connect(realExpression.y, chromoloxHeater_Power.u) annotation (Line(points={{
          -73,64},{-70,64},{-70,84},{-80,84},{-80,92},{-73.2,92}}, color={0,0,
          127}));
  connect(actuatorSubBus.Heater_Input, chromoloxHeater_Power.y) annotation (
      Line(
      points={{-31,106},{-50,106},{-50,92},{-59.4,92}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(sensor_m_flow3.port_a, T_discharge_outlet.port_b)
    annotation (Line(points={{154,30},{154,21}}, color={0,127,255}));
  connect(m_thot.port_b, TC_201.port_a)
    annotation (Line(points={{84,22},{84,19}}, color={0,127,255}));
  connect(TC_201.port_b, thermocline_Insulation.port_a)
    annotation (Line(points={{84,-5},{84,-6},{118,-6}}, color={0,127,255}));
  connect(FM_202.port_a, T_ch_o.port_a)
    annotation (Line(points={{154,-90},{154,-75}}, color={0,127,255}));
  connect(TC_202.port_b, thermocline_Insulation.port_b)
    annotation (Line(points={{80,-51},{80,-50},{118,-50}}, color={0,127,255}));
  connect(actuatorSubBus.T_discharge_outlet, T_discharge_outlet.T) annotation (
      Line(
      points={{-31,106},{202,106},{202,9},{158.68,9}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.T_discharge_inlet, TC_202.T) annotation (Line(
      points={{-31,106},{26,106},{26,-63},{75.32,-63}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.T_charge_outlet, T_ch_o.T) annotation (Line(
      points={{-31,106},{202,106},{202,-63},{158.68,-63}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.T_charge_inlet, TC_201.T) annotation (Line(
      points={{-31,106},{26,106},{26,7},{79.32,7}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.Charging_flowrate, FM_202.m_flow) annotation (Line(
      points={{-31,106},{202,106},{202,-101},{157.6,-101}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.Discharge_FlowRate, sensor_m_flow3.m_flow) annotation (
     Line(
      points={{-31,106},{202,106},{202,40},{157.6,40}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.Total_Mass_Flow_System, FM_002.m_flow) annotation (
      Line(
      points={{-31,106},{240,106},{240,-186},{126.6,-186}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.Heater_flowrate, FM_003.m_flow) annotation (Line(
      points={{-31,106},{-82,106},{-82,80},{-100,80},{-100,44},{-86,44},{-86,-32},
          {-80,-32},{-80,-128},{-100.12,-128}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(pipe2.port_b, BOP_Mass_flow.port_a)
    annotation (Line(points={{62,76},{96,76}}, color={0,127,255}));
  connect(BOP_Mass_flow.port_b,PV_006. port_a)
    annotation (Line(points={{116,76},{126,76}}, color={0,127,255}));
  connect(actuatorSubBus.heater_BOP_massflow, BOP_Mass_flow.m_flow) annotation (
     Line(
      points={{-31,106},{106,106},{106,79.6}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(sensor_m_flow2.port_b, T_chiller_before.port_a)
    annotation (Line(points={{228,38},{228,28}}, color={0,127,255}));
  connect(T_chiller_before.port_b, pipe7.port_a)
    annotation (Line(points={{228,2},{228,-10}}, color={0,127,255}));
  connect(pipe7.port_b, T_chiller_after.port_a) annotation (Line(points={{228,
          -26},{228,-52},{227,-52}}, color={0,127,255}));
  connect(T_chiller_after.port_b, pipe3.port_a) annotation (Line(points={{227,
          -78},{227,-142},{176,-142}}, color={0,127,255}));
  connect(tank1.port_b, TC_003a.port_a)
    annotation (Line(points={{7.6,75.2},{7.6,76},{18,76}}, color={0,127,255}));
  connect(TC_003a.port_b, pipe2.port_a)
    annotation (Line(points={{42,76},{46,76}}, color={0,127,255}));
  connect(Heater_BOP_Demand.y, P_001.in_p) annotation (Line(points={{-30.9,-169},
          {-30.9,-170},{6,-170},{6,-151.84},{-8,-151.84}}, color={0,0,127}));
  connect(const2.y, Chiller_Mass_Flow.m_flow_in) annotation (Line(points={{0.4,
          -228},{0.4,-226},{30,-226}},
                                 color={0,0,127}));
  connect(BVctr.y, PV_009.opening) annotation (Line(points={{97.1,-165},{97.1,
          -164},{98,-164},{98,-148.8},{92,-148.8}}, color={0,0,127}));
  connect(TC_006.T, MassFlow_Control1.u_m) annotation (Line(points={{40,-153.68},
          {40,-196},{-28,-196},{-28,-200.8}}, color={0,0,127}));
  connect(MassFlow_Control1.y, PV_012.opening) annotation (Line(points={{-21.4,
          -208},{46,-208},{46,-202},{51.2,-202}}, color={0,0,127}));
  connect(Glycol_HX.port_b_shell, nonLinear_Break.port_a) annotation (Line(
        points={{69,-222.1},{56,-222.1},{56,-222}}, color={0,127,255}));
  connect(nonLinear_Break.port_b, PV_012.port_a)
    annotation (Line(points={{56,-210},{56,-208}}, color={0,127,255}));
  connect(TC_006.port_a, nonLinear_Break2.port_a) annotation (Line(points={{28,
          -149},{28,-146},{22,-146}}, color={0,127,255}));
  connect(nonLinear_Break2.port_b, P_001.port_a) annotation (Line(points={{10,-146},
          {4.44089e-16,-146}}, color={0,127,255}));
  connect(V1.y[1],PV_049. opening) annotation (Line(points={{56.7,165},{62,165},
          {62,90},{74,90},{74,66},{79.2,66}}, color={0,0,127}));
  connect(V2.y[1],PV_006. opening) annotation (Line(points={{106.7,165},{112,
          165},{112,92},{132,92},{132,80.8}}, color={0,0,127}));
  connect(V3.y[1],PV_050. opening) annotation (Line(points={{148.7,165},{166,
          165},{166,66},{158.8,66}},                   color={0,0,127}));
  connect(sensor_m_flow1.port_a, P_001.port_b)
    annotation (Line(points={{-24,-146},{-16,-146}}, color={0,127,255}));
  connect(PV_004.port_b, ValveFl.port_b)
    annotation (Line(points={{-84,-146},{-60,-146}}, color={0,127,255}));
  connect(ValveFl.port_a, sensor_m_flow1.port_b)
    annotation (Line(points={{-48,-146},{-42,-146}}, color={0,127,255}));
  connect(Flow.y[1], VolFlow_Control2.u_s) annotation (Line(points={{-61.3,-73},
          {-61.3,-74},{-43.2,-74}},                     color={0,0,127}));
  connect(VolFlow_Control2.y, ValveFl.opening) annotation (Line(points={{-29.4,
          -74},{-20,-74},{-20,-100},{-54,-100},{-54,-141.2}},
        color={0,0,127}));
  connect(nonLinear_Break1.port_a,PV_004. port_b) annotation (Line(points={{-22,
          -126},{-74,-126},{-74,-146},{-84,-146}}, color={0,127,255}));
  connect(sensor_m_flow1.m_flow, product1.u2) annotation (Line(points={{-33,
          -143.12},{-34,-143.12},{-34,-114},{-140,-114},{-140,-120},{-160,-120},
          {-160,-114},{-155,-114}}, color={0,0,127}));
  connect(Heater_BOP_Demand2.y, product1.u1) annotation (Line(points={{-170.9,
          -111},{-170.9,-112},{-162,-112},{-162,-108},{-155,-108}},
                       color={0,0,127}));
  connect(GPMconversion.y, FM_001.u1) annotation (Line(points={{-170.9,-89},{-170.9,
          -90},{-127,-90}}, color={0,0,127}));
  connect(product1.y, FM_001.u2) annotation (Line(points={{-143.5,-111},{-143.5,
          -112},{-132,-112},{-132,-96},{-127,-96}}, color={0,0,127}));
  connect(THXout.y[1], add.u1) annotation (Line(points={{-97.3,-195},{-97.3,
          -202},{-86,-202}},                       color={0,0,127}));
  connect(const3.y, add.u2) annotation (Line(points={{-105.6,-214},{-86,-214}},
                                 color={0,0,127}));
  connect(add.y, MassFlow_Control1.u_s) annotation (Line(points={{-63,-208},{
          -35.2,-208}},                                             color={0,0,
          127}));
  connect(FM_001.y, VolFlow_Control2.u_m) annotation (Line(points={{-115.5,-93},
          {-115.5,-94},{-36,-94},{-36,-81.2}}, color={0,0,127}));
  connect(firstOrder5.y,PV_051. opening) annotation (Line(points={{99.4,-116},{
          84.8,-116}},                    color={0,0,127}));
  connect(V3.y[1], firstOrder5.u) annotation (Line(points={{148.7,165},{252,165},
          {252,-114},{182,-114},{182,-116},{113.2,-116}},                color=
          {0,0,127}));
  connect(PV_052.opening, V1.y[1]) annotation (Line(points={{158.8,-128},{266,
          -128},{266,182},{62,182},{62,165},{56.7,165}},
        color={0,0,127}));
  connect(const4.y, add1.u2) annotation (Line(points={{-185.6,78},{-174,78},{
          -174,76},{-166,76}}, color={0,0,127}));
  connect(THeater.y[1], add1.u1) annotation (Line(points={{-175.3,103},{-175.3,
          96},{-170,96},{-170,88},{-166,88}}, color={0,0,127}));
  connect(add1.y, Chromolox_Heater_Control.u_s) annotation (Line(points={{-143,
          82},{-130,82},{-130,46},{-123.2,46}}, color={0,0,127}));
  connect(Glycol_HX.port_a_shell, TC_004.port_a) annotation (Line(points={{100,
          -222.1},{124,-222.1},{124,-222}}, color={0,127,255}));
  connect(PV_012.port_b, TC_005.port_b)
    annotation (Line(points={{56,-196},{56,-170}}, color={0,127,255}));
  connect(TC_005.port_a, TC_006.port_b) annotation (Line(points={{56,-150},{56,
          -149},{52,-149}}, color={0,127,255}));
  connect(FM_002.port_b, TC_004.port_b) annotation (Line(points={{123,-199},{
          124,-199},{124,-202}}, color={0,127,255}));
  connect(nonLinear_Break3.port_b, PV_009.port_a) annotation (Line(points={{106,
          -142},{102,-142},{102,-144},{98,-144}}, color={0,127,255}));
  connect(p1.port_a, nonLinear_Break1.port_b) annotation (Line(points={{16,-122},
          {-2,-122},{-2,-126},{-10,-126}}, color={0,127,255}));
  connect(pipe5.port_a,PV_049. port_b)
    annotation (Line(points={{90,56},{84,56},{84,60}}, color={0,127,255}));
  connect(pipe5.port_b, m_thot.port_a) annotation (Line(points={{106,56},{112,
          56},{112,44},{84,44},{84,42}}, color={0,127,255}));
  connect(p1.port_b, FM_201.port_a)
    annotation (Line(points={{32,-122},{40,-122}}, color={0,127,255}));
  connect(FM_201.port_b, PV_051.port_b)
    annotation (Line(points={{64,-122},{80,-122}}, color={0,127,255}));
  connect(TC_202.port_a, pipe6.port_b)
    annotation (Line(points={{80,-75},{80,-80}}, color={0,127,255}));
  connect(pipe6.port_a, PV_051.port_a)
    annotation (Line(points={{80,-92},{80,-110}}, color={0,127,255}));
  connect(const1.y, PV_008.opening)
    annotation (Line(points={{139.6,-150},{128.8,-150}}, color={0,0,127}));
  connect(pipe3.port_b, nonLinear_Break3.port_a)
    annotation (Line(points={{164,-142},{118,-142}}, color={0,127,255}));
  connect(pipe3.port_b, PV_008.port_a) annotation (Line(points={{164,-142},{124,
          -142},{124,-144}}, color={0,127,255}));
  connect(TC_006.port_b, PV_009.port_b) annotation (Line(points={{52,-149},{52,
          -150},{56,-150},{56,-144},{86,-144}}, color={0,127,255}));
  connect(T_discharge_outlet.port_a, pipe1.port_b) annotation (Line(points={{
          154,-3},{154,-10},{140,-10},{140,16}}, color={0,127,255}));
  connect(pipe1.port_a, pipe5.port_b)
    annotation (Line(points={{140,32},{140,56},{106,56}}, color={0,127,255}));
  connect(PV_052.port_b, FM_002.port_a) annotation (Line(points={{154,-134},{
          154,-170},{123,-170},{123,-173}}, color={0,127,255}));
  connect(pipe8.port_b, T_ch_o.port_b) annotation (Line(points={{134,-76},{134,
          -51},{154,-51}}, color={0,127,255}));
  connect(pipe8.port_a, PV_051.port_a) annotation (Line(points={{134,-88},{134,
          -96},{80,-96},{80,-110}}, color={0,127,255}));
  connect(PV_008.port_b, p2.port_a) annotation (Line(points={{124,-156},{124,
          -160},{128,-160},{128,-164}}, color={0,127,255}));
  connect(p2.port_b, FM_002.port_a) annotation (Line(points={{144,-164},{154,
          -164},{154,-170},{123,-170},{123,-173}}, color={0,127,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-220},{240,
            140}}), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-156,-208},{232,138}}),
        Polygon(lineColor = {0,0,255},
                fillColor = {75,138,73},
                pattern = LinePattern.None,
                fillPattern = FillPattern.Solid,
                points={{-24,68},{176,-30},{-24,-150},{-24,68}})}),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-160,-220},{
            240,140}})),
    experiment(
      StopTime=16000,
      Interval=10,
      Tolerance=0.001,
      __Dymola_Algorithm="Esdirk45a"),
    __Dymola_Commands(file="../../TEDS/Basic_TEDS_setup.mos" "Basic_TEDS_setup",
        file="../../TEDS/M3_TEDS.mos" "M3_TEDS"),
    conversion(noneFromVersion=""));
end TEDSloop_allmodes_test_3WV_exp2023TTAdjTime3FV2_v2a;
