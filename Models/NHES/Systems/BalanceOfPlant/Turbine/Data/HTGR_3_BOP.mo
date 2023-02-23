within NHES.Systems.BalanceOfPlant.Turbine.Data;
model HTGR_3_BOP

  extends BaseClasses.Record_Data;

  //BOP General Parameters
      //Systems
      //Component Pressures
  parameter Modelica.Units.SI.Pressure p_condensor = 0.1e5 "Condensor pressure"
  annotation (Dialog(tab="General Parameters",group = "Pressures"));
  parameter Modelica.Units.SI.Pressure p_boundary = 100000 "Boundary pressure for venting"
  annotation (Dialog(tab="General Parameters", group= "Pressures"));
      //Component Volumes
  parameter Modelica.Units.SI.Volume  V_condensor = 2500 "Condensor volume"
  annotation (Dialog(tab="General Parameters", group = "Volumes"));
  parameter Modelica.Units.SI.Volume  V_tee1 = 5 "Tee volume"
  annotation (Dialog(tab="General Parameters", group = "Volumes"));
  parameter Modelica.Units.SI.Volume  V_tee2 = 5 "Tee volume"
  annotation (Dialog(tab="General Parameters", group = "Volumes"));


  //Valve Parameters
  parameter Modelica.Units.SI.MassFlowRate valve_TBV_mflow = 50 "Turbine Bypass valve nominal mass flow"
  annotation (Dialog(tab="Valves", group="Turbine Bypass Valve"));
  parameter Modelica.Units.SI.Pressure valve_TBV_dp_nominal = 1e5 "Nominal pressure drop for turbine Bypass"
  annotation (Dialog(tab="Valves", group="Turbine Bypass Valve"));
  parameter Modelica.Units.SI.MassFlowRate valve_TCV_mflow = 50 "Turbine Control valve nominal mass flow"
  annotation (Dialog(tab="Valves", group="Turbine Control Valve"));
  parameter Modelica.Units.SI.Pressure valve_TCV_dp_nominal = 1e5 "Nominal pressure drop for turbine control"
  annotation (Dialog(tab="Valves", group="Turbine Control Valve"));
  parameter Modelica.Units.SI.MassFlowRate valve_LPT1_Bypass_mflow = 30 "LPT1 Bypass valve nominal mass flow"
  annotation (Dialog(tab="Valves", group="LPT Bypass Valve"));
  parameter Modelica.Units.SI.Pressure valve_LPT1_Bypass_dp_nominal = 1e5 "Nominal pressure drop for LPT1 Bypass Valve"
  annotation (Dialog(tab="Valves", group="LPT Bypass Valve"));
  parameter Modelica.Units.SI.MassFlowRate valve_LPT2_Bypass_mflow = 20 "LPT2 Bypass valve nominal mass flow"
  annotation (Dialog(tab="Valves", group="Turbine External Bypass Valve"));
  parameter Modelica.Units.SI.Pressure valve_LPT2_Bypass_dp_nominal = 1e5 "Nominal pressure drop for LPT2 Bypass Valve"
  annotation (Dialog(tab="Valves", group="Turbine External Bypass Valve"));

  //Turbine Parameters
  parameter Modelica.Units.SI.Pressure HPT_p_in_nominal = 4138400 "Nominal HPT outlet pressure"
  annotation (Dialog(tab="Turbines", group="High Pressure Turbine"));
  parameter Modelica.Units.SI.Pressure HPT_p_exit_nominal = 407230 "Nominal HPT outlet pressure"
  annotation (Dialog(tab="Turbines", group="High Pressure Turbine"));
  parameter Modelica.Units.SI.Temperature HPT_T_in_nominal = 540.13+273.15 "Nominal HPT inlet temperature"
  annotation (Dialog(tab="Turbines", group="High Pressure Turbine"));
  parameter Modelica.Units.SI.Temperature HPT_T_exit_nominal = 253.77+273.15 "Nominal HPT inlet temperature"
  annotation (Dialog(tab="Turbines", group="High Pressure Turbine"));
  parameter Modelica.Units.SI.MassFlowRate HPT_nominal_mflow = 200 "HPT nominal mass flow"
  annotation (Dialog(tab="Turbines", group="High Pressure Turbine"));
  parameter Real HPT_efficiency = 0.93 "HPT mechanical efficiency"
  annotation (Dialog(tab="Turbines", group="High Pressure Turbine"));

  parameter Modelica.Units.SI.Pressure LPT1_p_in_nominal = 2000000 "Nominal LPT1 inlet pressure"
  annotation (Dialog(tab="Turbines", group="Low Pressure Turbine-1"));
  parameter Modelica.Units.SI.Pressure LPT1_p_exit_nominal = 1500000 "Nominal LPT1 outlet pressure"
  annotation (Dialog(tab="Turbines", group="Low Pressure Turbine-1"));
  parameter Modelica.Units.SI.Temperature LPT1_T_in_nominal = 150+273.15 "Nominal LPT1 inlet temperature"
  annotation (Dialog(tab="Turbines", group="Low Pressure Turbine-1"));
  parameter Modelica.Units.SI.Temperature LPT1_T_exit_nominal = 200+273.15 "Nominal LPT1 inlet temperature"
  annotation (Dialog(tab="Turbines", group="Low Pressure Turbine-1"));
  parameter Modelica.Units.SI.MassFlowRate LPT1_nominal_mflow = 200 "LPT1 nominal mass flow"
  annotation (Dialog(tab="Turbines", group="Low Pressure Turbine-1"));
  parameter Real LPT1_efficiency = 0.93 "LPT1 mechanical efficiency"
  annotation (Dialog(tab="Turbines", group="Low Pressure Turbine-1"));

  parameter Modelica.Units.SI.Pressure LPT2_p_in_nominal = 1500000 "Nominal LPT-2 inlet pressure"
  annotation (Dialog(tab="Turbines", group="Low Pressure Turbine-2"));
  parameter Modelica.Units.SI.Pressure LPT2_p_exit_nominal = 8000 "Nominal LPT-2 outlet pressure"
  annotation (Dialog(tab="Turbines", group="Low Pressure Turbine-2"));
  parameter Modelica.Units.SI.Temperature LPT2_T_in_nominal = 150+273.15 "Nominal LPT-2 inlet temperature"
  annotation (Dialog(tab="Turbines", group="Low Pressure Turbine-2"));
  parameter Modelica.Units.SI.Temperature LPT2_T_exit_nominal = 41.52+273.15 "Nominal LPT-2 inlet temperature"
  annotation (Dialog(tab="Turbines", group="Low Pressure Turbine-2"));
  parameter Modelica.Units.SI.MassFlowRate LPT2_nominal_mflow = 200 "LPT-2 nominal mass flow"
  annotation (Dialog(tab="Turbines", group="Low Pressure Turbine-2"));
  parameter Real LPT2_efficiency = 0.93 "LPT-2 mechanical efficiency"
  annotation (Dialog(tab="Turbines", group="Low Pressure Turbine-2"));



  //Pump Parameters
  parameter Modelica.Units.NonSI.AngularVelocity_rpm pump_feedWaterControl_RPM = 1200 "FWCP_Pump speed (RPM)"
  annotation (Dialog(tab="Pumps", group="Feedwater Control Pump"));
  parameter Modelica.Units.SI.Pressure pump_feedWaterControl_dp_nominal = 150e5 "FWCP_Pressure increase"
  annotation (Dialog(tab="Pumps", group="Feedwater Control Pump"));
  parameter Modelica.Units.SI.MassFlowRate pump_feedWaterControl_nominal_mflow = 50 "FWCP_Massflow rate"
  annotation (Dialog(tab="Pumps", group="Feedwater Control Pump"));
  parameter Modelica.Units.SI.Density pump_feedWaterControl_nominal_density = 1000 "FWCP_Density"
  annotation (Dialog(tab="Pumps", group="Feedwater Control Pump"));

  parameter Modelica.Units.SI.Pressure pump_feedWater_nominal_pressure = 55e5 "FWP_nominal outlet Pressure"
  annotation (Dialog(tab="Pumps", group="Feedwater Pump"));


  //Heat Exchangers
    //Bypass Feedwater Heater
  parameter Real BypassFeedHeater_NTU = 20 "NTU of bypass feedwater heater"
  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Real BypassFeedHeater_K_tube(unit = "1/m4") = 17000 "K value of tube in bypass feedwater heater"
  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Real BypassFeedHeater_K_shell(unit = "1/m4") = 500 "K value of shell in bypass feedwater heater"
  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.Volume  BypassFeedHeater_V_tube = 5 "Tube side volume in bypass feedwater heater"
  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.Volume  BypassFeedHeater_V_shell = 5 "Shell side volume in bypass feedwater heater"
  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));

  annotation (Dialog(tab="Turbines", group="Low Pressure Turbine"),
              Dialog(tab="System Setpoints"),
              Dialog(tab="System Setpoints"),
    defaultComponentName="data",
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
          lineColor={0,0,0},
          extent={{-100,-90},{100,-70}},
          textStyle={TextStyle.Bold},
          textString="HTGR-3-BOP")}),
    Diagram(coordinateSystem(preserveAspectRatio=false)));

end HTGR_3_BOP;
