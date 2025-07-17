-------------Design archive overview--------------

Phase 2 files: (preliminary ILC scripts)

Drone4into12:
This script runs the analytically derived UAV model from the report. Is not P2P enabled but has other ILC methods not used in the report.

p2p:
This is the p2p enabled version of Drone4into12, also has other methods that are p2p enabled but where never tested.

ILCLinear:
This script applies the updates of Drone4into12 onto the Simulink model linearmod

linearmod:
Simulink model of the state space model analytical derived in the report.


Phase 3 files: (sim modelling)

CrazyflieSimulationPID_20ablock2:
This is the Simulink model of the crazyfly drone used for testing in the report. no modifications should be needed to this file to run tests

CrazyflueSimulationParameters:
file of Simulink parameters that needs to be loaded before running crazyvlockV2.

CrazyP2P:
this script can run either NOILC or p2p ILC. this was the main script used for testing.

Tuning_script:
This script is essentially CrazyP2P but has been modified to be used in Q_tuner and R_tuner but can be used for the same tests conducted in CrazyP2P

Q_tuner and R_tuner:
Automated testing of range of Q and R values respectively. Both generate a mydata.mat file for each test.







