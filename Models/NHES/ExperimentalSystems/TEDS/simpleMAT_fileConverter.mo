within NHES.ExperimentalSystems.TEDS;
function simpleMAT_fileConverter
  "Function to import trajectory result files and write them as MatLab compatible .mat files. To use within Python use commands:
  =======
  import scipy.io
  mat = scipy.io.loadmat('file.mat')
  =======
  to import the .mat file in as a python dictionary
  "


input String filename="TEDSloop_allmodes_test_3WV_exp2023TTAdjTime3FV2.mat" "File to be converted" annotation (Dialog(__Dymola_loadSelector(filter="Matlab files (*.mat)",
caption="Select the results trajectory file")));


input String varOrigNames[:]={"Time",
                              "thermocline_Insulation.thermocline_fluidprops_heaters_newHC_120C.Tf[28]",
                              "thermocline_Insulation.thermocline_fluidprops_heaters_newHC_120C.Tf[100]",
                              "thermocline_Insulation.thermocline_fluidprops_heaters_newHC_120C.Tf[172]",
                              "T_discharge_Inlet.T",
                              "T_Charge_Inlet.T",
                              "HX_exit_temperature_T66.T",+
                              "T_inlet_HX.T"} "Variable names/headers in the file in modelica syntax";

input String varReNames[:]={"Time",
                            "Simul_TN_1_2",
                            "Simul_TW_1_1",
                            "Simul_TE_3_1",
                            "Simul_TC_202",
                            "Simul_TC_201",
                            "Simul_TC_006",
                            "Simul_TC_004"} "Variable names which will appear in the MAT results file";

input String outputFilename="valveArea_060.mat";

protected
   Integer noRows "Number of rows in the trajectory being converted";
   Integer noColumn=8 "Number of columns in the trajectory being converted";
   Real data[:,:] "Data read in from trajectory file";
   Real dataDump[:,:] "Sacrificial dump variable for writeMatrix command";
   Integer i=2 "Loop counter";

algorithm
   noRows := DymolaCommands.Trajectories.readTrajectorySize(filename);
   data := DymolaCommands.Trajectories.readTrajectory(
     filename,
     varOrigNames,
     noRows);
data := transpose(data);
noColumn := size(data, 2);
while i <= noColumn loop
   dataDump := [data[:, 1],data[:, i]];
   if i == 2 then
     DymolaCommands.MatrixIO.writeMatrix(
       outputFilename,
       varReNames[i],
       dataDump);
   else
      DymolaCommands.MatrixIO.writeMatrix(
       outputFilename,
       varReNames[i],
       dataDump,
       true);
   end if;
i := i + 1;
end while;
annotation (Documentation(info="<html>
<p></p>
</html>"), uses(DymolaCommands(version="1.4")));
end simpleMAT_fileConverter;
