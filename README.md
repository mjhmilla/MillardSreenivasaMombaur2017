# MillardSreenivasaMombaur2019
Contains a predictive simulation of lifting a box with and without the aid of an exo-skeleton

# Description

This repository contains the RBDL model and MUSCOD-II OCP used to perform the simulations documented in Millard, Sreenivasa, and Mombaur (2017). Here is a brief overview of where to find various resources associated with this project:

- Human experimental lifting data and models
	- Models 
		- simulation/OCP/PRE/vuaHuman2d.lua
		- simulation/OCP/PRE/vuaHuman2d.15kgBox_BaumgarteDisabled.lua 
		- simulation/OCP/PRE/vuaHuman3d.lua	
	- Data: See ID, IK
	- Note: VUA2UHEI contains data needed to map data from the Vrije Universitat Amsterdam model to the RBDL multibody model 
- Simulation of lifting 
	- Models used in animation: 
		- postprocessing/media/vuaHuman2dRightyExo_15kgBox_BaumgarteEnabled.lua
	- Models used in pre-processing experimental data:
		- simulation/OCP/PRE/vuaHuman2d.lua, 
		- simulation/OCP/PRE/vuaHuman2d_15kgBox_BaumgarteDisabled.lua 
		- simulation/OCP/PREvuaHuman3d.lua
	- Models used in simulation:
		- simulation/OCP/DAT/vuaHuman2dRightyExo_15kgBox_BaumgarteEnabled.lua 
		- simulation/OCP/DAT/vuaHuman2dRighty_15kgBox_BaumgarteDisabled.lua

Millard M, Sreenivasa M, Mombaur K. Predicting the motions and forces of wearable robotic systems using optimal control. Frontiers in Robotics and AI. 2017 Aug 30;4:41. https://doi.org/10.3389/frobt.2017.00041

