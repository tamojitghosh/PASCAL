Signal Control Codes

This repository contains the implementation codes for five different signal control models:

* PASCAL
* Max-Pressure (MP) control by Varaiya (2013)
* MP control by Le et al. (2015)
* MP control by Levin et al. (2019)
* Capacity-Aware MP

All codes are developed using the Aimsun API, and therefore they can only be executed within the Aimsun environment.

Repository structure

For each signal control model, there are five separate code files, with each file corresponding to one specific intersection in the study network.

Within each file, the split links for that intersection are identified separately for the upstream links and downstream links. These split link IDs are provided through two dictionaries defined in the code. For a given intersection, these split link IDs remain the same across the files of different signal control models.

Model-specific parameters

The remaining model parameters are defined directly within each code file. These include:

* intersection ID
* minimum green time
* maximum green time
* time step
* amber time
* all-red time (clearance red)
* space headway

Accordingly, each file is self-contained for the corresponding intersection and signal control model implementation.

Important note

These codes are intended for implementation and testing within Aimsun only. They are not standalone Python scripts and require the appropriate Aimsun API environment to run.
