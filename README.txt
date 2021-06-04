## README
This README.txt file will give an overview of the matlab code provided as Final.m. The 
code designes an LQR controller for a selected linearised model of a slackliner and
provides plots to compare the step response to a PP controller. 

## Installation instructions
- Select the controller in the State Space model section. State 'P = on; I = off;' 
  for the model derived by Paoletti or 'P = off; I = on;' for the model derived 
  by Iqbal (as used in the paper). 
- Hit the 'Run' botton. 

## Sections
- Variables: contains the variables to obtain the model of the slackliner. These values 
  are ajustable. Also, the sample time to for plotting is defined here. 
- State Space model: Here the state space model of Iqbal, et al. and Paoletti, et al. are
  represented. State 'P = on; I = off;' for the model derived by Paoletti or 
  'P = off; I = on;' for the model derived by Iqbal (as used in the paper). 
- Checking for...: will check for stability, controllability and observability and provides
  an output on whether this is the case or not. 
- LQR controller: here, the controller is designed for the selected model. 
- PP conroller: here, the benchmark controller is desinged for the selected model. A gain 
  is chosen here according to the Iterative process, but the gain is ajustable. 
- Performance: the results are plot here. 

### Error messege
When the messege 'No state space' comes across, make sure the state space is selected in 
a correct way. State 'P = on; I = off;' for the model derived by Paoletti or 
'P = off; I = on;' for the model derived by Iqbal (as used in the paper). 

## Version

The R2020a version of MATLAB is used. 

## Authors

Melissa Molenaars, student of the Technical University of Delft. 

## Date 

03-06-2021



