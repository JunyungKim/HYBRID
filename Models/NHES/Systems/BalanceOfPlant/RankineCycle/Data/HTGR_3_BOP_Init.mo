within NHES.Systems.BalanceOfPlant.RankineCycle.Data;
record HTGR_3_BOP_Init

  extends TRANSFORM.Icons.Record;

    //Bypass Feedwater Heater
  parameter Modelica.Units.SI.Pressure BypassFeedHeater_tube_p_start = 55e5 "Initial Tube pressure of bypass feedwater heater"   annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.Pressure BypassFeedHeater_shell_p_start = 10e5 "Initial Shell pressure of bypass feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));

  parameter Modelica.Units.SI.SpecificEnthalpy BypassFeedHeater_h_start_tube_inlet = 1e6 "Initial Tube inlet specific enthalpy of main feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.SpecificEnthalpy BypassFeedHeater_h_start_tube_outlet = 1.05e6 "Initial Tube outlet specific enthalpy of main feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.SpecificEnthalpy BypassFeedHeater_h_start_shell_inlet = 3e6 "Initial Shell inlet specific enthalpy of main feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.SpecificEnthalpy BypassFeedHeater_h_start_shell_outlet = 2.9e6 "Initial Shell outlet specific enthalpy of main feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.Temperature BypassFeedHeater_tube_T_start_inlet = 45+273 "Initial Tube inlet temperature of bypass feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.Temperature BypassFeedHeater_tube_T_start_outlet = 200+273 "Initial Tube outlet temperature of bypass feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.Temperature BypassFeedHeater_shell_T_start_inlet = 370+273 "Initial Tube inlet temperature of bypass feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.Temperature BypassFeedHeater_shell_T_start_outlet = 250+273 "Initial Tube outlet temperature of bypass feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.Pressure BypassFeedHeater_dp_init_tube = 0 "Initial Tube pressure drop of bypass feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.Pressure BypassFeedHeater_dp_init_shell = 100000 "Initial Shell pressure drop of bypass feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.MassFlowRate BypassFeedHeater_m_start_tube = 72 "Initial tube mass flow rate in bypass feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.MassFlowRate BypassFeedHeater_m_start_shell = 10 "Initial shell mass flow rate in main feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));
  parameter Modelica.Units.SI.Power  BypassFeedHeater_Q_init = 1e6 "Initial Heat Flow in main feedwater heater"  annotation (Dialog(tab="Heat Exchangers", group="Bypass Feedwater Heater"));

  parameter Modelica.Units.SI.Volume  V_condensor_liquid_start = 1.2 "Condensor volume"
  annotation (Dialog(tab="Condensor", group = "Volumes"));

  // HPT initial conditions
  parameter SI.Pressure HPT_P_inlet =     4.1384e6 annotation (Evaluate=true, Dialog(tab="Turbines", group="HPT_initial_conditions"));
parameter SI.Pressure HPT_P_outlet =    4.0723e5 annotation (Evaluate=true, Dialog(tab="Turbines", group="HPT_initial_conditions"));
parameter SI.Temperature HPT_T_inlet =  540.13 + 273.15 annotation (Evaluate=true, Dialog(tab="Turbines", group="HPT_initial_conditions"));
parameter SI.Temperature HPT_T_outlet = 253.77 + 273.15 annotation (Evaluate=true, Dialog(tab="Turbines", group="HPT_initial_conditions"));

  // LPT1 initial conditions
parameter SI.Pressure LPT1_P_inlet =     30e5 annotation (Evaluate=true, Dialog(tab="Turbines", group="LPT1_initial_conditions"));
parameter SI.Pressure LPT1_P_outlet =    15e5 annotation (Evaluate=true, Dialog(tab="Turbines", group="LPT1_initial_conditions"));
parameter SI.Temperature LPT1_T_inlet =  300 + 273.15 annotation (Evaluate=true, Dialog(tab="Turbines", group="LPT1_initial_conditions"));
parameter SI.Temperature LPT1_T_outlet = 200 + 273.15 annotation (Evaluate=true, Dialog(tab="Turbines", group="LPT1_initial_conditions"));

  // LPT2 initial conditions
parameter SI.Pressure LPT2_P_inlet =     15e5 annotation (Evaluate=true, Dialog(tab="Turbines", group="LPT2_initial_conditions"));
parameter SI.Pressure LPT2_P_outlet =    0.08e5 annotation (Evaluate=true, Dialog(tab="Turbines", group="LPT2_initial_conditions"));
parameter SI.Temperature LPT2_T_inlet =  250 + 273.15 annotation (Evaluate=true, Dialog(tab="Turbines", group="LPT2_initial_conditions"));
parameter SI.Temperature LPT2_T_outlet =  70 + 273.15 annotation (Evaluate=true, Dialog(tab="Turbines", group="LPT2_initial_conditions"));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                                                Text(
          lineColor={0,0,0},
          extent={{-100,-90},{100,-70}},
          textString="HTGR-3-BOP",
          textStyle={TextStyle.Bold})}),                         Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end HTGR_3_BOP_Init;
