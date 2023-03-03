within NHES.Systems.BalanceOfPlant;
package HRSG "Heat Recovery Steam Generators"
  model HRSG "Single pressure heat recovery steam generator"

   parameter Boolean use_nat_circ = false
      "Make unit free convection"
      annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
   parameter Boolean use_booster_pump = false
      "Add pump to boost feed to a higher pressure HRSG"
      annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
   parameter Boolean use_Econimizer = false
      "Add econimizer to heat inlet feed water"
      annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
  parameter Boolean use_Superheater = false
      "Add superheater to heat exit steam"
      annotation(Evaluate=true, HideResult=true, choices(checkBox=true));

      parameter Modelica.Units.SI.MassFlowRate Recirculation_Rate= 175.5;

   parameter Modelica.Units.SI.Pressure P_sys "System Pressure";
   parameter Modelica.Units.SI.Pressure Nominal_dp=(P_sys/1000) "Nominal Pressure drop";
   parameter Modelica.Units.SI.MassFlowRate Nominal_Flow=(Recirculation_Rate.*0.1);
   parameter Real PCV_opening_start=0.5
                                       "PCV start postion";
   parameter Real YMAX= (if use_booster_pump then 100 else 1) "Three Element Controler max output (0-1 for valve positon, or max booster pump flow rate)";

  parameter Real n_tubes_EVAP=20 "Number of Tubes in Evaporator"
    annotation (Dialog(tab="Evaporator Dimensions",
          group="Evaporator Tubes"));
  parameter Modelica.Units.SI.Length EVAP_L=10 "Height of Evaporator"
    annotation (Dialog(tab="Evaporator Dimensions",
          group="Evaporator Tubes"));
          parameter Modelica.Units.SI.Length EVAP_tube_Dia=50e-3  "Hydraulic Diameter of Tubes"
    annotation (Dialog(tab="Evaporator Dimensions",
          group="Evaporator Tubes"));
  parameter Modelica.Units.SI.Length EVAP_shell_Dia=300e-3  "Hydraulic Diameter of Shell"
    annotation (Dialog(tab="Evaporator Dimensions",
          group="Evaporator Shell"));
  parameter Real n_tubes_EVAP_DC=1 "Number of Downcomers in Evaporator"
  annotation (Dialog(tab="Evaporator Dimensions",
          group="Evaporator Downcomer"));
  parameter Modelica.Units.SI.Length EVAP_DC_Dia=0.75  "Diameter of Downcomer"
    annotation (Dialog(tab="Evaporator Dimensions",
          group="Evaporator Downcomer"));

  parameter Real n_tubes_ECON=20 "Number of Tubes in Econimizer"
    annotation (Dialog(tab="Econimizer Dimensions",
          group="Econimizer Tubes"));
          parameter Modelica.Units.SI.Length ECON_L=10  "Height of Econimizer"
    annotation (Dialog(tab="Econimizer Dimensions",
          group="Econimizer Tubes"));
          parameter Modelica.Units.SI.Length ECON_tube_Dia=50e-3 "Hydraulic Diameter of Tubes"
    annotation (Dialog(tab="Econimizer Dimensions",
          group="Econimizer Tubes"));
          parameter Modelica.Units.SI.Length ECON_shell_Dia=300e-3  "Hydraulic Diameter of Shell"
    annotation (Dialog(tab="Econimizer Dimensions",
          group="Econimizer Shell"));
          parameter Real n_tubes_ECON_DC=1  "Number of Downcomers in Econimizer"
  annotation (Dialog(tab="Econimizer Dimensions",
          group="Econimizer Downcomer"));
  parameter Modelica.Units.SI.Length ECON_DC_Dia=0.75 "Diameter of Downcomer"
    annotation (Dialog(tab="Econimizer Dimensions",
          group="Econimizer Downcomer"));

  parameter Real n_tubes_SH=20 "Number of Tubes in Superheater"
    annotation (Dialog(tab="Superheater Dimensions",
          group="Superheater Tubes"));
          parameter Modelica.Units.SI.Length SH_L=10  "Height of Superheater"
    annotation (Dialog(tab="Superheater Dimensions",
          group="Superheater Tubes"));
          parameter Modelica.Units.SI.Length SH_tube_Dia=50e-3  "Hydraulic Diameter of Tubes"
    annotation (Dialog(tab="Superheater Dimensions",
          group="Superheater Tubes"));
          parameter Modelica.Units.SI.Length SH_shell_Dia=300e-3  "Hydraulic Diameter of Shell"
    annotation (Dialog(tab="Superheater Dimensions",
          group="Superheater Shell"));
          parameter Real n_tubes_SH_DC=1  "Number of Downcomers in Superheater"
  annotation (Dialog(tab="Superheater Dimensions",
          group="Superheater Downcomer"));
          parameter Modelica.Units.SI.Length SH_DC_Dia=0.75 "Diameter of Downcomer"
    annotation (Dialog(tab="Superheater Dimensions",
          group="Superheater Downcomer"));

    Modelica.Fluid.Interfaces.FluidPort_a Feed_port(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-10,-170},{10,-150}}),  iconTransformation(extent={{-10,-170},{10,-150}})));
    Modelica.Fluid.Interfaces.FluidPort_a Gas_Inlet_Port(redeclare package
        Medium =
          NHES.Media.FlueGas)
      annotation (Placement(transformation(extent={{150,-10},{170,10}}), iconTransformation(extent={{150,-10},{170,10}})));
    Modelica.Fluid.Interfaces.FluidPort_b Gas_Outlet_Port(redeclare package
        Medium =
          NHES.Media.FlueGas)
      annotation (Placement(transformation(extent={{-170,-10},{-150,10}}), iconTransformation(extent={{-170,-10},{-150,10}})));
    Modelica.Fluid.Interfaces.FluidPort_b Steam_Outlet_port(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater) annotation (Placement(transformation(extent={{-10,150},{10,170}}), iconTransformation(extent={{-10,150},
              {10,170}})));
    Modelica.Fluid.Sensors.MassFlowRate massFlowRate(redeclare package Medium =
          Modelica.Media.Water.StandardWater)
      annotation (Placement(transformation(extent={{-104,28},{-84,48}})));
    Modelica.Fluid.Sensors.MassFlowRate massFlowRate1(redeclare package Medium =
          Modelica.Media.Water.StandardWater,                                                                        allowFlowReversal=false)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={82,70})));
    Modelica.Fluid.Valves.ValveLinear valveLinear(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      allowFlowReversal=true,
      m_flow_start=1,
      dp_nominal=50000,
      m_flow_nominal=50) if not use_booster_pump
      annotation (Placement(transformation(extent={{-110,68},{-90,88}})));
    TRANSFORM.Fluid.Pipes.GenericPipe_MultiTransferSurface Downcomer(
      nParallel=n_tubes_EVAP_DC,
      redeclare package Medium = Modelica.Media.Water.WaterIF97_ph,
      ps_start=TRANSFORM.Math.linspace_1D(
          Downcomer.p_a_start,
          Downcomer.p_b_start,
          Downcomer.nV),
      use_Ts_start=false,
      p_a_start(displayUnit="bar") = P_sys*1.2,
      p_b_start(displayUnit="bar") = P_sys*1.2,
      T_a_start=548.15,
      h_a_start=1400e3,
      h_b_start=1400e3,
      m_flow_a_start=Recirculation_Rate,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.StraightPipe
          (
          dimension(displayUnit="mm") = EVAP_DC_Dia,
          length=EVAP_L,
          angle=-1.5707963267949,
          nV=3,
          height_a=10),
      redeclare model FlowModel =
          TRANSFORM.Fluid.ClosureRelations.PressureLoss.Models.DistributedPipe_1D.SinglePhase_Developed_2Region_NumStable)
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-14,-42})));
    ThreeElementContoller                    threeElementContoller(
      FlowErrorGain=0.01,
      k=100,
      yMax=YMAX,
      yMin=0,
      kp=250,
      LevelSet=0.5,
      xi_start=0,
      Ti=1)
      annotation (Placement(transformation(extent={{8,80},{-16,102}})));
    TRANSFORM.HeatExchangers.GenericDistributed_HX Evaporator(
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.HeatExchanger.StraightPipeHX
          (
          nV=5,
          nTubes=n_tubes_EVAP,
          nR=3,
          height_a_tube=0,
          dimension_shell(displayUnit="mm") = EVAP_shell_Dia,
          length_shell=EVAP_L,
          dimension_tube(displayUnit="mm") = EVAP_tube_Dia,
          length_tube=EVAP_L,
          angle_tube=1.5707963267949),
      redeclare package Medium_shell = NHES.Media.FlueGas,
      redeclare package Medium_tube = Modelica.Media.Water.StandardWater,
      redeclare package Material_tubeWall = TRANSFORM.Media.Solids.SS304,
      redeclare model FlowModel_shell =
          TRANSFORM.Fluid.ClosureRelations.PressureLoss.Models.DistributedPipe_1D.SinglePhase_Turbulent_MSL,
      redeclare model FlowModel_tube =
          TRANSFORM.Fluid.ClosureRelations.PressureLoss.Models.DistributedPipe_1D.TwoPhase_Developed_2Region_NumStable,
      redeclare model HeatTransfer_tube =
          TRANSFORM.Fluid.ClosureRelations.HeatTransfer.Models.DistributedPipe_1D_MultiTransferSurface.Alphas_TwoPhase_3Region,
      p_a_start_shell=200000,
      p_b_start_shell=120000,
      T_a_start_shell=773.15,
      m_flow_a_start_shell=200,
      p_a_start_tube=P_sys*1.2,
      p_b_start_tube=P_sys*1.2,
      use_Ts_start_tube=false,
      T_a_start_tube=548.15,
      h_a_start_tube=1400e3,
      h_b_start_tube=1550e3,
      m_flow_a_start_tube=Recirculation_Rate)
                               annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=90,
          origin={26,-44})));

    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance EvaporatorResistance(redeclare
        package Medium =
          Modelica.Media.Water.StandardWater, R=0.1)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={30,-2})));
    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance DowncomerResistance(redeclare
        package Medium =
          Modelica.Media.Water.StandardWater, R=0.1)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-14,-14})));
    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance Feed_Resistance(redeclare
        package Medium =
          Modelica.Media.Water.StandardWater, R=0.1)                                                                                               if not use_Econimizer
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-58,38})));
    Fluid.Vessels.Steam_Drum
                    steam_Drum(
      p_start=P_sys,
      alphag_start=0.5,
      V_drum=10)
      annotation (Placement(transformation(extent={{16,30},{36,50}})));
    TRANSFORM.HeatExchangers.GenericDistributed_HX Economizer(
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.HeatExchanger.StraightPipeHX
          (
          nV=5,
          nTubes=n_tubes_ECON,
          nR=3,
          dimension_shell(displayUnit="mm") = ECON_shell_Dia,
          length_shell=ECON_L,
          dimension_tube(displayUnit="mm") = ECON_tube_Dia,
          length_tube=ECON_L),
      redeclare package Medium_shell = NHES.Media.FlueGas,
      redeclare package Medium_tube = Modelica.Media.Water.StandardWater,
      redeclare package Material_tubeWall = TRANSFORM.Media.Solids.SS304,
      redeclare model FlowModel_shell =
          TRANSFORM.Fluid.ClosureRelations.PressureLoss.Models.DistributedPipe_1D.SinglePhase_Turbulent_MSL,
      redeclare model HeatTransfer_shell =
          TRANSFORM.Fluid.ClosureRelations.HeatTransfer.Models.DistributedPipe_1D_MultiTransferSurface.Nus_SinglePhase_2Region,
      redeclare model FlowModel_tube =
          TRANSFORM.Fluid.ClosureRelations.PressureLoss.Models.DistributedPipe_1D.SinglePhase_Turbulent_MSL,
      redeclare model HeatTransfer_tube =
          TRANSFORM.Fluid.ClosureRelations.HeatTransfer.Models.DistributedPipe_1D_MultiTransferSurface.Alphas_TwoPhase_3Region,
      p_a_start_shell=200000,
      p_b_start_shell=120000,
      T_a_start_shell=673.15,
      m_flow_a_start_shell=200,
      p_a_start_tube=P_sys*1.2,
      p_b_start_tube=P_sys*1.2,
      use_Ts_start_tube=false,
      T_a_start_tube=548.15,
      h_a_start_tube=400e3,
      h_b_start_tube=400e3,
      m_flow_a_start_tube=0)  if use_Econimizer  annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=90,
          origin={-56,-36})));
    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance Gas_Resistance(redeclare
        package Medium =
          NHES.Media.FlueGas,                                                                                                R=5) if not use_Econimizer
                                   annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-76,-110})));
    TRANSFORM.Fluid.Pipes.GenericPipe_MultiTransferSurface ECON_Downcomer(
      nParallel=n_tubes_ECON_DC,
      redeclare package Medium = Modelica.Media.Water.WaterIF97_ph,
      ps_start=TRANSFORM.Math.linspace_1D(
          Downcomer.p_a_start,
          Downcomer.p_b_start,
          Downcomer.nV),
      use_Ts_start=false,
      p_a_start(displayUnit="bar") = P_sys*1.2,
      p_b_start(displayUnit="bar") = P_sys*1.2,
      T_a_start=548.15,
      h_a_start=400e3,
      h_b_start=400e3,
      m_flow_a_start=0,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.StraightPipe
          (
          dimension(displayUnit="mm") = ECON_DC_Dia,
          length=ECON_L,
          nV=3),
      redeclare model FlowModel =
          TRANSFORM.Fluid.ClosureRelations.PressureLoss.Models.DistributedPipe_1D.SinglePhase_Developed_2Region_NumStable)
                                                                                                                                                   if use_Econimizer
      annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={-74,-42})));
    TRANSFORM.HeatExchangers.GenericDistributed_HX Superheater(
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.HeatExchanger.StraightPipeHX
          (
          nV=5,
          nTubes=n_tubes_SH,
          nR=3,
          dimension_shell(displayUnit="mm") = SH_shell_Dia,
          length_shell=SH_L,
          dimension_tube(displayUnit="mm") = SH_tube_Dia,
          length_tube=SH_L),
      redeclare package Medium_shell = NHES.Media.FlueGas,
      redeclare package Medium_tube = Modelica.Media.Water.StandardWater,
      redeclare package Material_tubeWall = TRANSFORM.Media.Solids.SS304,
      redeclare model FlowModel_shell =
          TRANSFORM.Fluid.ClosureRelations.PressureLoss.Models.DistributedPipe_1D.SinglePhase_Turbulent_MSL,
      redeclare model FlowModel_tube =
          TRANSFORM.Fluid.ClosureRelations.PressureLoss.Models.DistributedPipe_1D.SinglePhase_Turbulent_MSL,
      redeclare model HeatTransfer_tube =
          TRANSFORM.Fluid.ClosureRelations.HeatTransfer.Models.DistributedPipe_1D_MultiTransferSurface.Nus_SinglePhase_2Region_modelBased,
      p_a_start_shell=200000,
      p_b_start_shell=120000,
      T_a_start_shell=773.15,
      m_flow_a_start_shell=200,
      p_a_start_tube=P_sys*1.2,
      p_b_start_tube=P_sys*1.2,
      use_Ts_start_tube=false,
      T_a_start_tube=548.15,
      h_a_start_tube=2724e3,
      h_b_start_tube=2724e3,
      m_flow_a_start_tube=Nominal_Flow)
                              if use_Superheater  annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=90,
          origin={82,-36})));

    TRANSFORM.Fluid.Pipes.GenericPipe_MultiTransferSurface SH_Downcomer(
      nParallel=n_tubes_SH_DC,
      redeclare package Medium = Modelica.Media.Water.WaterIF97_ph,
      ps_start=TRANSFORM.Math.linspace_1D(
          Downcomer.p_a_start,
          Downcomer.p_b_start,
          Downcomer.nV),
      use_Ts_start=false,
      p_a_start(displayUnit="bar") = P_sys*1.2,
      p_b_start(displayUnit="bar") = P_sys*1.2,
      T_a_start=548.15,
      h_a_start=2724e3,
      h_b_start=2724e3,
      m_flow_a_start=50,
      redeclare model Geometry =
          TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.StraightPipe
          (
          dimension(displayUnit="mm") = SH_DC_Dia,
          length=SH_L,
          nV=3),
      redeclare model FlowModel =
          TRANSFORM.Fluid.ClosureRelations.PressureLoss.Models.DistributedPipe_1D.SinglePhase_Developed_2Region_NumStable)
                                                                                                                                                   if use_Superheater
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={54,-38})));
    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance Gas_Resistance_2(redeclare
        package Medium =
          NHES.Media.FlueGas,                                                                                                  R=5) if not use_Superheater
                                   annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={126,-112})));
    TRANSFORM.Fluid.Machines.Pump_SimpleMassFlow Booster_Pump(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater,
      use_input=true,                                                                                                        m_flow_nominal=20)
                                                                                                                                         if use_booster_pump
      annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={-124,50})));
    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance Steam_Resistance(redeclare
        package Medium =
          Modelica.Media.Water.StandardWater, R=0.1)                                                                                 if not use_Superheater
                                   annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={64,48})));
    NHES.Fluid.Valves.PressureCV             pressure_Control_Valve_Simple(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
        Pressure_target=P_sys, Use_input=false,
      ValvePos_start=PCV_opening_start,
      init_time=PCV_init_time,
      PID_k=-1e-4)
      annotation (Placement(transformation(extent={{48,120},{28,140}})));
    TRANSFORM.Fluid.Machines.Pump_SimpleMassFlow RecircPump(redeclare package
        Medium =
          Modelica.Media.Water.StandardWater,                                                                              m_flow_nominal=
          Recirculation_Rate) if not use_nat_circ annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={4,-84})));
    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance NatCirc(redeclare
        package Medium =
          Modelica.Media.Water.StandardWater, R=0.1)                                                                                       if use_nat_circ
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={6,-66})));
    parameter SI.Time PCV_init_time=0 "Time instant of step start";
  equation

    connect(massFlowRate.m_flow,threeElementContoller. FeedFlow) annotation (Line(
          points={{-94,49},{-94,58},{2,58},{2,77.8}},   color={0,0,127}));
    connect(threeElementContoller.SteamFlow,massFlowRate1. m_flow) annotation (Line(
          points={{-10,77.8},{-10,70},{71,70}},                       color={0,0,127}));
    connect(threeElementContoller.y,valveLinear. opening)
      annotation (Line(points={{-17.2,91},{-100,91},{-100,86}},
                                                              color={0,0,127}));
    connect(Evaporator.port_b_tube,EvaporatorResistance. port_a)
      annotation (Line(points={{26,-34},{26,-16},{30,-16},{30,-9}},
                                                   color={244,125,35}));
    connect(Downcomer.port_a,DowncomerResistance. port_a)
      annotation (Line(points={{-14,-32},{-14,-21}}, color={244,125,35}));
    connect(Feed_Resistance.port_a, steam_Drum.feed_port) annotation (Line(
          points={{-51,38},{-18,38},{-18,40},{16,40}}, color={0,127,255}));
    connect(EvaporatorResistance.port_b, steam_Drum.riser_port)
      annotation (Line(points={{30,5},{30,30}}, color={244,125,35}));
    connect(steam_Drum.RelLevel, threeElementContoller.Level) annotation (Line(
          points={{36.2,40},{38,40},{38,91},{10.4,91}}, color={0,0,127}));
    connect(Evaporator.port_b_shell,Gas_Resistance. port_a)
      annotation (Line(points={{30.6,-54},{30.6,-96},{-60,-96},{-60,-110},{-69,-110}},
                                                                 color={0,140,72}));
    connect(valveLinear.port_b,massFlowRate. port_a)
      annotation (Line(points={{-90,78},{-84,78},{-84,62},{-108,62},{-108,38},{-104,38}},
                                                     color={0,127,255}));
    connect(ECON_Downcomer.port_b,Economizer. port_a_tube)
      annotation (Line(points={{-74,-52},{-74,-58},{-56,-58},{-56,-46}},     color={0,127,255}));
    connect(SH_Downcomer.port_b,Superheater. port_a_tube)
      annotation (Line(points={{54,-48},{54,-52},{82,-52},{82,-46}}, color={238,46,47}));
    connect(Evaporator.port_a_shell,Gas_Resistance_2. port_b)
      annotation (Line(points={{30.6,-34},{30.6,-24},{40,-24},{40,-82},{60,-82},{60,-112},{119,-112}},
                                                                                  color={0,140,72}));
    connect(Gas_Inlet_Port, Superheater.port_a_shell) annotation (Line(points={{160,0},{154,0},{154,-20},{86.6,-20},{86.6,-26}},
                                                                                                                   color={0,140,72}));
    connect(Gas_Outlet_Port, Economizer.port_b_shell)
      annotation (Line(points={{-160,0},{-118,0},{-118,-72},{-51.4,-72},{-51.4,
            -46}},                                                                      color={0,140,72}));
    connect(Feed_port, Booster_Pump.port_a) annotation (Line(points={{0,-160},{0,-148},{-148,-148},{-148,50},{-134,50}},
                                                                                                     color={0,127,255}));
    connect(Booster_Pump.in_m_flow, threeElementContoller.y)
      annotation (Line(points={{-124,57.3},{-124,91},{-17.2,91}},                                         color={0,0,127}));
    connect(Feed_port, valveLinear.port_a) annotation (Line(points={{0,-160},{0,-148},{-148,-148},{-148,78},{-110,78}}, color={0,127,255}));
    connect(Booster_Pump.port_b, massFlowRate.port_a) annotation (Line(points={{-114,50},{-108,50},{-108,38},{-104,38}}, color={0,127,255}));
    connect(SH_Downcomer.port_a, steam_Drum.steam_port)
      annotation (Line(points={{54,-28},{54,50},{26,50}}, color={238,46,47}));
    connect(massFlowRate.port_b, ECON_Downcomer.port_a) annotation (Line(points={{-84,38},{-74,38},{-74,-32}}, color={0,127,255}));
    connect(Superheater.port_b_tube, massFlowRate1.port_a) annotation (Line(points={{82,-26},{82,60}},            color={238,46,47}));
    connect(massFlowRate.port_b, Feed_Resistance.port_b) annotation (Line(points={{-84,38},{-65,38}}, color={0,127,255}));
    connect(steam_Drum.steam_port, Steam_Resistance.port_b) annotation (Line(
          points={{26,50},{42,50},{42,48},{57,48}}, color={238,46,47}));
    connect(Steam_Resistance.port_a, massFlowRate1.port_a) annotation (Line(points={{71,48},{82,48},{82,60}},
                                                                                                        color={238,46,47}));
    connect(Economizer.port_b_tube, steam_Drum.feed_port) annotation (Line(
          points={{-56,-26},{-56,40},{16,40}}, color={0,127,255}));
    connect(Gas_Resistance.port_b, Gas_Outlet_Port) annotation (Line(points={{-83,-110},{-118,-110},{-118,0},{-160,0}},     color={0,140,72}));
    connect(Superheater.port_b_shell, Evaporator.port_a_shell)
      annotation (Line(points={{86.6,-46},{86.6,-54},{40,-54},{40,-24},{30.6,-24},{30.6,-34}},   color={0,140,72}));
    connect(Gas_Resistance_2.port_a, Gas_Inlet_Port) annotation (Line(points={{133,-112},{154,-112},{154,0},{160,0}},     color={0,140,72}));
    connect(massFlowRate1.port_b, pressure_Control_Valve_Simple.port_a)
      annotation (Line(points={{82,80},{82,130},{48,130}}, color={238,46,47}));
    connect(pressure_Control_Valve_Simple.port_b, Steam_Outlet_port)
      annotation (Line(points={{28,130},{0,130},{0,160}}, color={238,46,47}));
    connect(steam_Drum.downcomer_port, DowncomerResistance.port_b) annotation (
        Line(points={{22,30},{22,4},{-14,4},{-14,-7}}, color={244,125,35}));
    connect(Economizer.port_a_shell, Evaporator.port_b_shell)
      annotation (Line(points={{-51.4,-26},{-52,-26},{-52,-24},{-44,-24},{-44,
            -96},{30.6,-96},{30.6,-54}},                                                                   color={0,140,72}));
    connect(Downcomer.port_b, RecircPump.port_a) annotation (Line(points={{-14,-52},{-14,-84},{-6,-84}}, color={0,127,255}));
    connect(RecircPump.port_b, Evaporator.port_a_tube) annotation (Line(points={{14,-84},{26,-84},{26,-54}}, color={0,127,255}));
    connect(Downcomer.port_b, NatCirc.port_b) annotation (Line(points={{-14,-52},{-14,-66},{-1,-66}}, color={0,127,255}));
    connect(NatCirc.port_a, Evaporator.port_a_tube) annotation (Line(points={{13,-66},{26,-66},{26,-54}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},{160,160}}), graphics={
          Rectangle(
            extent={{-160,60},{160,-60}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-23,3},{23,-3}},
            lineColor={0,0,0},
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5,
            origin={-1,-103},
            rotation=180),
          Rectangle(
            extent={{-99,4},{99,-4}},
            lineColor={0,0,0},
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5,
            origin={-20,-7},
            rotation=90),
          Rectangle(
            extent={{-99,4},{99,-4}},
            lineColor={0,0,0},
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5,
            origin={20,-7},
            rotation=90),
          Ellipse(
            extent={{-40,80},{40,160}},
            lineColor={0,0,0},
            lineThickness=1,
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-40,80},{40,160}},
            lineColor={0,0,0},
            lineThickness=1,
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-40,80},{40,160}},
            lineColor={0,0,0},
            lineThickness=1,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            closure=EllipseClosure.Chord,
            startAngle=10,
            endAngle=170),
          Text(
            extent={{-100,-120},{100,-140}},
            textColor={28,108,200},
            textString="%name")}),                                 Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},{160,160}})),
      experiment(
        StopTime=100000,
        Interval=0.1,
        __Dymola_Algorithm="Esdirk45a"));
  end HRSG;

  package Examples

    model HRSG_test
      extends Modelica.Icons.Example;
    Modelica.Fluid.Sources.Boundary_pT SinkExhaustGas(
            p=10000,
            redeclare package Medium = NHES.Media.FlueGas,
            use_p_in=false,
            nPorts=1)           annotation (Placement(transformation(extent={{74,42},
                    {90,58}},
                          rotation=0)));
          Modelica.Fluid.Sources.Boundary_pT FeedSource(
            p=500000,
            redeclare package Medium = Modelica.Media.Water.StandardWater,
            use_p_in=false,
            T=313.15,
            nPorts=1) annotation (Placement(transformation(extent={{78,-72},{62,-56}},   rotation=0)));
          Modelica.Fluid.Sources.Boundary_pT SteamSink(
            p=100000,
            redeclare package Medium = Modelica.Media.Water.StandardWater,
            use_p_in=false,
            nPorts=1) annotation (Placement(transformation(extent={{94,78},{78,94}},   rotation=0)));
          NHES.Systems.BalanceOfPlant.HRSG.HRSG heatRecoverySteamGenerator(
            use_nat_circ=false,
            use_booster_pump=true,
            use_Econimizer=true,
            use_Superheater=false,
            Recirculation_Rate=10,
        P_sys=862000,
            Nominal_Flow=5.5,
            YMAX=5,
            n_tubes_EVAP=500,
            EVAP_tube_Dia(displayUnit="mm") = 0.03,
            EVAP_shell_Dia(displayUnit="mm") = 0.15,
            EVAP_DC_Dia=0.3,
            n_tubes_ECON=10,
            ECON_L=6,
            ECON_tube_Dia(displayUnit="mm") = 0.05,
            ECON_shell_Dia(displayUnit="mm") = 0.75,
            ECON_DC_Dia=0.3,
            threeElementContoller(
              k=100,
              kp=1e5,
              Ti=2.5))
            annotation (Placement(transformation(extent={{58,-40},{-22,40}})));
          TRANSFORM.Fluid.BoundaryConditions.MassFlowSource_T boundary(
            redeclare package Medium = NHES.Media.FlueGas,
            m_flow=10,
            T=973.15,
            nPorts=1)
            annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
    equation
          connect(heatRecoverySteamGenerator.Steam_Outlet_port, SteamSink.ports[1])
            annotation (Line(points={{18,40},{18,86},{78,86}},  color={0,127,255}));
          connect(heatRecoverySteamGenerator.Gas_Outlet_Port, SinkExhaustGas.ports[1]) annotation (Line(points={{58,0},{
                  96,0},{96,50},{90,50}},                                                                                           color={0,127,255}));
          connect(heatRecoverySteamGenerator.Feed_port, FeedSource.ports[1])
            annotation (Line(points={{18,-40},{18,-64},{62,-64}},         color={0,127,255}));
          connect(boundary.ports[1], heatRecoverySteamGenerator.Gas_Inlet_Port)
            annotation (Line(points={{-52,0},{-22,0}}, color={0,127,255}));
          annotation (
            Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                    120,100}})),
            Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                    {100,100}})),
            experiment(
              StopTime=1000,
              Interval=1,
              __Dymola_Algorithm="Esdirk45a"));
    end HRSG_test;
    annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Polygon(
            origin={8,14},
            lineColor={78,138,73},
            fillColor={78,138,73},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}));
  end Examples;

  model ThreeElementContoller

    parameter Real FlowErrorGain(unit="1")=100 "Gain of Flow Error";
    parameter Real LevelErrorGain(unit="1")=1 "Gain of Level Error";
    parameter Real k(min=0, unit="1") = 1 "Gain of controller";
    parameter Real yMax(start=1) "Upper limit of output";
    parameter Real yMin(start=0) "Lower limit of output";
    parameter Real kp(min=0) = 1 "Proportional Gain";
    parameter Real DelayTime = 2 "Time Delay for start of controller";
    parameter Real SS_guess = 0 "Init Guess for SS before start of controller";
    parameter Real LevelSet(min=0, max=1)=0.5
                                             "Relative set point for level";
  parameter Real xi_start=0
      "Initial or guess value for integrator output (= integrator state)";
    parameter Modelica.Units.SI.Time Ti(min=Modelica.Constants.small)=0.5;
    parameter Real Ni(min=100*Modelica.Constants.eps) = 0.9
      "Ni*Ti is time constant of anti-windup compensation";
    constant Modelica.Units.SI.Time unitTime=1;
    Modelica.Blocks.Interfaces.RealInput Level
      annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
    Modelica.Blocks.Interfaces.RealInput FeedFlow annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-50,-120})));
    Modelica.Blocks.Interfaces.RealInput SteamFlow annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={50,-120})));
    Modelica.Blocks.Interfaces.RealOutput y
      annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    Modelica.Blocks.Sources.RealExpression LevelSetPoint(y=LevelSet)
      annotation (Placement(transformation(extent={{-100,-26},{-80,-6}})));
    Modelica.Blocks.Math.Add LevelError(k1=-1)
      annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
    Modelica.Blocks.Math.Add FlowError(k1=+1, k2=-1)
                                              annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=-90,
          origin={0,-80})));
    Modelica.Blocks.Math.Add TotalError(k1=FlowErrorGain, k2=LevelErrorGain)
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=-90,
          origin={-30,-50})));
    Modelica.Blocks.Math.Gain P(k=kp)
      annotation (Placement(transformation(extent={{-80,80},{-60,100}})));
    Modelica.Blocks.Continuous.Integrator I(k=unitTime/Ti, y_start=xi_start)
      annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
    Modelica.Blocks.Math.Add addPI(k1=1, k2=1)
      annotation (Placement(transformation(extent={{0,60},{20,80}})));
    Modelica.Blocks.Math.Gain gainPID(k=k)
      annotation (Placement(transformation(extent={{30,60},{50,80}})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=yMax, uMin=yMin)
      annotation (Placement(transformation(extent={{60,60},{80,80}})));
    Modelica.Blocks.Logical.Switch DelaySwitch annotation (Placement(transformation(extent={{60,10},{80,-10}})));
    Modelica.Blocks.Sources.BooleanStep DelayTimer(startTime=DelayTime, startValue=true)
                                                                        annotation (Placement(transformation(extent={{20,0},{40,20}})));
    Modelica.Blocks.Sources.RealExpression SteadyStateGuess(y=SS_guess) annotation (Placement(transformation(extent={{20,-30},{40,-10}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder(T=1)
      annotation (Placement(transformation(extent={{88,-62},{108,-42}})));
  equation
    connect(LevelSetPoint.y, LevelError.u2)
      annotation (Line(points={{-79,-16},{-72,-16},{-72,-6}}, color={0,0,127}));
    connect(Level, LevelError.u1)
      annotation (Line(points={{-120,0},{-80,0},{-80,6},{-72,6}}, color={0,0,127}));
    connect(FeedFlow, FlowError.u2) annotation (Line(points={{-50,-120},{-50,-96},{-6,-96},
            {-6,-92}}, color={0,0,127}));
    connect(SteamFlow, FlowError.u1)
      annotation (Line(points={{50,-120},{50,-96},{6,-96},{6,-92}}, color={0,0,127}));
    connect(LevelError.y, TotalError.u2) annotation (Line(points={{-49,0},{-44,0},{-44,-68},
            {-36,-68},{-36,-62}}, color={0,0,127}));
    connect(I.y, addPI.u2)
      annotation (Line(points={{-19,50},{-8,50},{-8,64},{-2,64}}, color={0,0,127}));
    connect(P.y, addPI.u1)
      annotation (Line(points={{-59,90},{-8,90},{-8,76},{-2,76}}, color={0,0,127}));
    connect(addPI.y, gainPID.u)
      annotation (Line(points={{21,70},{28,70}}, color={0,0,127}));
    connect(gainPID.y, limiter.u)
      annotation (Line(points={{51,70},{58,70}}, color={0,0,127}));
    connect(P.u, TotalError.y) annotation (Line(points={{-82,90},{-88,90},{-88,18},{
            -28,18},{-28,-30},{-30,-30},{-30,-39}}, color={0,0,127}));
    connect(I.u, TotalError.y) annotation (Line(points={{-42,50},{-66,50},{-66,48},{
            -88,48},{-88,18},{-28,18},{-28,-30},{-30,-30},{-30,-39}}, color={0,0,127}));
    connect(FlowError.y, TotalError.u1) annotation (Line(points={{2.05391e-15,-69},{2.05391e-15,-64},{-18,-64},{-18,-68},
            {-24,-68},{-24,-62}}, color={0,0,127}));
    connect(DelayTimer.y, DelaySwitch.u2) annotation (Line(points={{41,10},{50,10},{50,0},{58,0}}, color={255,0,255}));
    connect(limiter.y, DelaySwitch.u3) annotation (Line(points={{81,70},{86,70},{86,14},{58,14},{58,8}}, color={0,0,127}));
    connect(SteadyStateGuess.y, DelaySwitch.u1) annotation (Line(points={{41,-20},{52,-20},{52,-8},{58,-8}}, color={0,0,127}));
    connect(DelaySwitch.y, firstOrder.u) annotation (Line(points={{81,0},{86,
            0},{86,-38},{76,-38},{76,-52},{86,-52}}, color={0,0,127}));
    connect(firstOrder.y, y) annotation (Line(points={{109,-52},{114,-52},{
            114,-14},{96,-14},{96,0},{110,0}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                  Rectangle(
          extent={{-100,-100},{100,100}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid), Text(
          extent={{-150,150},{150,110}},
          textColor={0,0,255},
            textString="%name"),
          Line(points={{-80,78},{-80,-90}}, color={192,192,192}),
          Polygon(
            points={{-80,90},{-88,68},{-72,68},{-80,90}},
            lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Line(points={{-90,-80},{82,-80}}, color={192,192,192}),
          Polygon(
            points={{90,-80},{68,-72},{68,-88},{90,-80}},
            lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Line(points={{-80,-80},{-80,-20},{30,60},{80,60}}, color={0,0,127}),
          Text(
            extent={{-20,-20},{80,-60}},
            textColor={192,192,192},
            textString="3EC")}),                                   Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end ThreeElementContoller;

end HRSG;
