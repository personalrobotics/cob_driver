/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_undercarriage_ctrl
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: April 2010:
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <cob_undercarriage_ctrl/UndercarriageCtrlGeom.h>

// Constructor
UndercarriageCtrlGeom::UndercarriageCtrlGeom()
{
	
	// init EMStop flag
	m_bEMStopActive = false;
	
	if (n.hasParam("NumberOfWheels"))
      	{
        	n.getParam("NumberOfWheels", m_iNumberOfDrives);
        	ROS_INFO("NumberOfWheels loaded from Parameter-Server is: %i", m_iNumberOfDrives);
      	}
      	else
      	{
        	ROS_WARN("No parameter NumberOfWheels on Parameter-Server. Using default: 4");
        	m_iNumberOfDrives = 4;
      	}

	// init vectors
	m_vdVelGearDriveRadS.assign(4,0);
	m_vdVelGearSteerRadS.assign(4,0);
	m_vdDltAngGearDriveRad.assign(4,0);
	m_vdAngGearSteerRad.assign(4,0);

	//m_vdVelGearDriveIntpRadS.assign(4,0);
	//m_vdVelGearSteerIntpRadS.assign(4,0);
	//m_vdAngGearSteerIntpRad.assign(4,0);

	m_vdVelGearDriveCmdRadS.assign(4,0);
	m_vdVelGearSteerCmdRadS.assign(4,0);
	m_vdAngGearSteerCmdRad.assign(4,0);

	m_vdWheelXPosMM.assign(4,0);
	m_vdWheelYPosMM.assign(4,0);
	m_vdWheelDistMM.assign(4,0);
	m_vdWheelAngRad.assign(4,0);

	m_vdExWheelXPosMM.assign(4,0);
	m_vdExWheelYPosMM.assign(4,0);
	m_vdExWheelDistMM.assign(4,0);
	m_vdExWheelAngRad.assign(4,0);

	m_vdAngGearSteerTarget1Rad.assign(4,0);
	m_vdVelGearDriveTarget1RadS.assign(4,0);
	m_vdAngGearSteerTarget2Rad.assign(4,0);
	m_vdVelGearDriveTarget2RadS.assign(4,0);
	m_vdAngGearSteerTargetRad.assign(4,0);
	m_vdVelGearDriveTargetRadS.assign(4,0);

	m_dCmdVelLongMMS = 0;
	m_dCmdVelLatMMS = 0;
	m_dCmdRotRobRadS = 0;
	m_dCmdRotVelRadS = 0;
	
	m_UnderCarriagePrms.WheelNeutralPos.assign(4,0);
	m_UnderCarriagePrms.vdSteerDriveCoupling.assign(4,0);
	m_UnderCarriagePrms.vdFactorVel.assign(4,0);
	
	m_vdCtrlVal.assign( 4, std::vector<double> (2,0.0) );
	
	
	// init Prms of Impedance-Ctrlr
	m_dSpring = 10.0;
	m_dDamp = 2.5;
	m_dVirtM = 0.1;
	m_dDPhiMax = 12.0;
	m_dDDPhiMax = 100.0;



}

// Destructor
UndercarriageCtrlGeom::~UndercarriageCtrlGeom(void)
{
		/*fclose(m_pfileDesVel);
		fclose(m_pfileMeasVel);

		fclose(m_pfileSteerAngTarget1);
		fclose(m_pfileSteerAngTarget2);

		fclose(m_pfileSteerAngTarget);
		fclose(m_pfileDriveVelTarget);

		fclose(m_pfileSteerAngCmd);
		fclose(m_pfileSteerVelCmd);
		fclose(m_pfileDriveVelCmd);*/
}

// Initialize Parameters for Controller and Kinematics
void UndercarriageCtrlGeom::InitUndercarriageCtrl(void)
{
	//Geom Parameter
	if (n.hasParam("Geom/DistWheels"))
      	{
        	n.getParam("Geom/DistWheels", m_UnderCarriagePrms.iDistWheels);
      	}
	if (n.hasParam("Geom/RadiusWheel"))
      	{
        	n.getParam("Geom/RadiusWheel", m_UnderCarriagePrms.iRadiusWheelMM);
      	}
	if (n.hasParam("Geom/DistSteerAxisToDriveWheelCenter"))
      	{
        	n.getParam("Geom/DistSteerAxisToDriveWheelCenter", m_UnderCarriagePrms.iDistSteerAxisToDriveWheelMM);
      	}
	if (n.hasParam("Geom/Wheel1XPos"))
      	{
        	n.getParam("Geom/Wheel1XPos", m_vdWheelXPosMM[0]);
      	}
	if (n.hasParam("Geom/Wheel1YPos"))
      	{
        	n.getParam("Geom/Wheel1YPos", m_vdWheelXPosMM[0]);
      	}
	if (n.hasParam("Geom/Wheel2XPos"))
      	{
        	n.getParam("Geom/Wheel2XPos", m_vdWheelXPosMM[1]);
      	}
	if (n.hasParam("Geom/Wheel2YPos"))
      	{
        	n.getParam("Geom/Wheel2YPos", m_vdWheelXPosMM[1]);
      	}
	if (n.hasParam("Geom/Wheel3XPos"))
      	{
        	n.getParam("Geom/Wheel3XPos", m_vdWheelXPosMM[2]);
      	}
	if (n.hasParam("Geom/Wheel3YPos"))
      	{
        	n.getParam("Geom/Wheel3YPos", m_vdWheelXPosMM[2]);
      	}
	if (n.hasParam("Geom/Wheel4XPos"))
      	{
        	n.getParam("Geom/Wheel4XPos", m_vdWheelXPosMM[3]);
      	}
	if (n.hasParam("Geom/Wheel4YPos"))
      	{
        	n.getParam("Geom/Wheel4YPos", m_vdWheelXPosMM[3]);
      	}
	//DrivePrms
	if (n.hasParam("DrivePrms/MaxDriveRate"))
      	{
        	n.getParam("DrivePrms/MaxDriveRate", m_UnderCarriagePrms.dMaxDriveRateRadpS);
      	}
	if (n.hasParam("DrivePrms/MaxSteerRate"))
      	{
        	n.getParam("DrivePrms/MaxSteerRate", m_UnderCarriagePrms.dMaxSteerRateRadpS);
      	}
	if (n.hasParam("DrivePrms/Wheel1SteerDriveCoupling"))
      	{
        	n.getParam("DrivePrms/Wheel1SteerDriveCoupling", m_UnderCarriagePrms.vdSteerDriveCoupling[0]);
      	}
	if (n.hasParam("DrivePrms/Wheel2SteerDriveCoupling"))
      	{
        	n.getParam("DrivePrms/Wheel2SteerDriveCoupling", m_UnderCarriagePrms.vdSteerDriveCoupling[1]);
      	}
	if (n.hasParam("DrivePrms/Wheel3SteerDriveCoupling"))
      	{
        	n.getParam("DrivePrms/Wheel3SteerDriveCoupling", m_UnderCarriagePrms.vdSteerDriveCoupling[2]);
      	}
	if (n.hasParam("DrivePrms/Wheel4SteerDriveCoupling"))
      	{
        	n.getParam("DrivePrms/Wheel4SteerDriveCoupling", m_UnderCarriagePrms.vdSteerDriveCoupling[3]);
      	}
	if (n.hasParam("DrivePrms/Wheel1NeutralPosition"))
      	{
        	n.getParam("DrivePrms/Wheel1NeutralPosition", m_UnderCarriagePrms.WheelNeutralPos[0]);
      	}
	if (n.hasParam("DrivePrms/Wheel2NeutralPosition"))
      	{
        	n.getParam("DrivePrms/Wheel2NeutralPosition", m_UnderCarriagePrms.WheelNeutralPos[1]);
      	}
	if (n.hasParam("DrivePrms/Wheel3NeutralPosition"))
      	{
        	n.getParam("DrivePrms/Wheel3NeutralPosition", m_UnderCarriagePrms.WheelNeutralPos[2]);
      	}
	if (n.hasParam("DrivePrms/Wheel4NeutralPosition"))
      	{
        	n.getParam("DrivePrms/Wheel4NeutralPosition", m_UnderCarriagePrms.WheelNeutralPos[3]);
      	}
	//Thread
	if (n.hasParam("Thread/ThrUCarrCycleTimeS"))
      	{
        	n.getParam("Thread/ThrUCarrCycleTimeS", m_UnderCarriagePrms.dCmdRateS);
      	}
	//Motion Control
	if (n.hasParam("SteerCtrl/Spring"))
      	{
        	n.getParam("SteerCtrl/Spring", m_dSpring);
      	}
	if (n.hasParam("SteerCtrl/Damp"))
      	{
        	n.getParam("SteerCtrl/Damp", m_dDamp);
      	}
	if (n.hasParam("SteerCtrl/VirtMass"))
      	{
        	n.getParam("SteerCtrl/VirtMass", m_dVirtM);
      	}
	if (n.hasParam("SteerCtrl/DPhiMax"))
      	{
        	n.getParam("SteerCtrl/DPhiMax", m_dDPhiMax);
      	}
	if (n.hasParam("SteerCtrl/DDPhiMax"))
      	{
        	n.getParam("SteerCtrl/DDPhiMax", m_dDDPhiMax);
      	}
	for(int i = 0; i<4; i++)
	{	
		m_UnderCarriagePrms.WheelNeutralPos[i] = MathSup::convDegToRad(m_UnderCarriagePrms.WheelNeutralPos[i]);
		// provisorial --> skip interpolation
		m_vdAngGearSteerCmdRad[i] = m_UnderCarriagePrms.WheelNeutralPos[i];
		//m_vdAngGearSteerIntpRad[i] = m_UnderCarriagePrms.WheelNeutralPos[i];
		
		// also Init choosen Target angle
		m_vdAngGearSteerTargetRad[i] = m_UnderCarriagePrms.WheelNeutralPos[i];
	}
	

	// calculate polar coords of Wheel Axis in robot coordinate frame
	for(int i=0; i<4; i++)
	{
		m_vdWheelDistMM[i] = sqrt( (m_vdWheelXPosMM[i] * m_vdWheelXPosMM[i]) + (m_vdWheelYPosMM[i] * m_vdWheelYPosMM[i]) );
		m_vdWheelAngRad[i] = MathSup::atan4quad(m_vdWheelXPosMM[i], m_vdWheelYPosMM[i]);
	}

	// Calculate exact position of wheels in cart. and polar coords in robot coordinate frame
	CalcExWheelPos();

	// calculate compensation factor for velocity
	for(int i = 0; i<4; i++)
	{
		m_UnderCarriagePrms.vdFactorVel[i] = - m_UnderCarriagePrms.vdSteerDriveCoupling[i]
				     +(double(m_UnderCarriagePrms.iDistSteerAxisToDriveWheelMM) / double(m_UnderCarriagePrms.iRadiusWheelMM));
	}

}

// Set desired value for Plattfrom Velocity to UndercarriageCtrl (Sollwertvorgabe)
void UndercarriageCtrlGeom::SetDesiredPltfVelocity(double dCmdVelLongMMS, double dCmdVelLatMMS, double dCmdRotRobRadS, double dCmdRotVelRadS)
{	
	// declare auxiliary variables
	double dCurrentPosWheelRAD;
	double dtempDeltaPhi1RAD, dtempDeltaPhi2RAD;	// difference between possible steering angels and current steering angle
	double dtempDeltaPhiCmd1RAD, dtempDeltaPhiCmd2RAD;	// difference between possible steering angels and last target steering angle
	double dtempWeightedDelta1RAD, dtempWeightedDelta2RAD; // weighted Summ of the two distance values

	// copy function parameters to member variables
	m_dCmdVelLongMMS = dCmdVelLongMMS;
	m_dCmdVelLatMMS = dCmdVelLatMMS;
	m_dCmdRotRobRadS = dCmdRotRobRadS;
	m_dCmdRotVelRadS = dCmdRotVelRadS;

	CalcInverse();

	// determine optimal Pltf-Configuration
	for (int i = 0; i<4; i++)
	{
		// Normalize Actual Wheel Position before calculation
		dCurrentPosWheelRAD = m_vdAngGearSteerRad[i];
		MathSup::normalizePi(dCurrentPosWheelRAD);
		
		// Calculate differences between current config to possible set-points
		dtempDeltaPhi1RAD = m_vdAngGearSteerTarget1Rad[i] - dCurrentPosWheelRAD;
		dtempDeltaPhi2RAD = m_vdAngGearSteerTarget2Rad[i] - dCurrentPosWheelRAD;
		MathSup::normalizePi(dtempDeltaPhi1RAD);
		MathSup::normalizePi(dtempDeltaPhi2RAD);
		// Calculate differences between last steering target to possible set-points
		dtempDeltaPhiCmd1RAD = m_vdAngGearSteerTarget1Rad[i] - m_vdAngGearSteerTargetRad[i];
		dtempDeltaPhiCmd2RAD = m_vdAngGearSteerTarget2Rad[i] - m_vdAngGearSteerTargetRad[i];
		MathSup::normalizePi(dtempDeltaPhiCmd1RAD);
		MathSup::normalizePi(dtempDeltaPhiCmd2RAD);
		
		// determine optimal setpoint value
		// 1st which set point is closest to current cinfog
		//     but: avoid permanent switching (if next target is about PI/2 from current config)
		// 2nd which set point is closest to last set point
		// "fitness criteria" to choose optimal set point:
		// calculate accumulted (+ weighted) difference between targets, current config. and last command
		dtempWeightedDelta1RAD = 0.6*fabs(dtempDeltaPhi1RAD) + 0.4*fabs(dtempDeltaPhiCmd1RAD);
		dtempWeightedDelta2RAD = 0.6*fabs(dtempDeltaPhi2RAD) + 0.4*fabs(dtempDeltaPhiCmd2RAD);

		// check which set point "minimizes fitness criteria"
		if (dtempWeightedDelta1RAD <= dtempWeightedDelta2RAD)
		{
			// Target1 is "optimal"
			m_vdVelGearDriveTargetRadS[i] = m_vdVelGearDriveTarget1RadS[i];
			m_vdAngGearSteerTargetRad[i] = m_vdAngGearSteerTarget1Rad[i];
		}
		else
		{
			// Target2 is "optimal"
			m_vdVelGearDriveTargetRadS[i] = m_vdVelGearDriveTarget2RadS[i];
			m_vdAngGearSteerTargetRad[i] = m_vdAngGearSteerTarget2Rad[i];
		}
		
		// provisorial --> skip interpolation and always take Target1
		//m_vdVelGearDriveCmdRadS[i] = m_vdVelGearDriveTarget1RadS[i];
		//m_vdAngGearSteerCmdRad[i] = m_vdAngGearSteerTarget1Rad[i];

		/*// interpolate between last setpoint and theone of the new setpoint, which is closest to the current configuration
		if (fabs(dtempDeltaPhi1RAD) <= fabs(dtempDeltaPhi2RAD))
		{
			// difference between new target orientation and last (interpolated) target orientation
			dtempDeltaPhi1RAD = m_vdAngGearSteerTarget1Rad[i] - m_vdAngGearSteerIntpRad[i];
			MathSup::normalizePi(dtempDeltaPhi1RAD);

			// calculate interpolation step sizes, to reach target at end of the cycle
			m_vdDeltaAngIntpRad[i] = dtempDeltaPhi1RAD;
			m_vdDeltaDriveIntpRadS[i] = (m_vdVelGearDriveTarget1RadS[i] - m_vdVelGearDriveIntpRadS[i]);

			// additionally calculate meen change in angular config for feedforward cmd
			//m_vdVelGearSteerIntpRadS[i] = dtempDeltaPhi1RAD/m_UnderCarriagePrms.dCmdRateS;
		}
		else
		{
			// difference between new target orientation and last (interpolated) target orientation
			dtempDeltaPhi2RAD = m_vdAngGearSteerTarget2Rad[i] - m_vdAngGearSteerIntpRad[i];
			MathSup::normalizePi(dtempDeltaPhi2RAD);

			// calculate interpolation step sizes, to reach target at end of the cycle
			m_vdDeltaAngIntpRad[i] = dtempDeltaPhi2RAD;
			m_vdDeltaDriveIntpRadS[i] = (m_vdVelGearDriveTarget2RadS[i] - m_vdVelGearDriveIntpRadS[i]);

			// additionally calculate meen change in angular config for feedforward cmd
			//m_vdVelGearSteerIntpRadS[i] = dtempDeltaPhi2RAD/m_UnderCarriagePrms.dCmdRateS;
		}*/
	}

	/*// Logging for debugging	
	// get current time
	m_RawTime.SetNow();
	m_dNowTime = m_RawTime - m_StartTime;
	// Log out Pltf-Velocities
	fprintf(m_pfileDesVel, "%f %f %f %f \n", m_dNowTime, dCmdVelLongMMS, dCmdVelLatMMS, dCmdRotRobRadS);
	fprintf(m_pfileMeasVel, "%f %f %f %f \n", m_dNowTime, m_dVelLongMMS, m_dVelLatMMS, m_dRotRobRadS);
	// Log out corresponding Joint-Configuration
	fprintf(m_pfileSteerAngTarget1, "%f %f %f %f %f \n", m_dNowTime, m_vdAngGearSteerTarget1Rad[0], m_vdAngGearSteerTarget1Rad[1], m_vdAngGearSteerTarget1Rad[2], m_vdAngGearSteerTarget1Rad[3]);
	fprintf(m_pfileSteerAngTarget2, "%f %f %f %f %f \n", m_dNowTime, m_vdAngGearSteerTarget2Rad[0], m_vdAngGearSteerTarget2Rad[1], m_vdAngGearSteerTarget2Rad[2], m_vdAngGearSteerTarget2Rad[3]);
	fprintf(m_pfileSteerAngTarget, "%f %f %f %f %f \n", m_dNowTime, m_vdAngGearSteerTargetRad[0], m_vdAngGearSteerTargetRad[1], m_vdAngGearSteerTargetRad[2], m_vdAngGearSteerTargetRad[3]);
	fprintf(m_pfileDriveVelTarget, "%f %f %f %f %f \n", m_dNowTime, m_vdVelGearDriveTargetRadS[0], m_vdVelGearDriveTargetRadS[1], m_vdVelGearDriveTargetRadS[2], m_vdVelGearDriveTargetRadS[3]);*/
}

// Set actual values of wheels (steer/drive velocity/position) (Istwerte)
void UndercarriageCtrlGeom::SetActualWheelValues(std::vector<double> vdVelGearDriveRadS, std::vector<double> vdVelGearSteerRadS, std::vector<double> vdDltAngGearDriveRad, std::vector<double> vdAngGearSteerRad)
{
	//LOG_OUT("Set Wheel Position to Controller");

	m_vdVelGearDriveRadS = vdVelGearDriveRadS;
	m_vdVelGearSteerRadS = vdVelGearSteerRadS;
	m_vdDltAngGearDriveRad = vdDltAngGearDriveRad;
	m_vdAngGearSteerRad = vdAngGearSteerRad;

	// calc exact Wheel Positions (taking into account lever arm)
	CalcExWheelPos();
	
	// Peform calculation of direct kinematics (approx.) based on corrected Wheel Positions
	CalcDirect();
}

// Get result of inverse kinematics (without controller)
void UndercarriageCtrlGeom::GetSteerDriveSetValues(std::vector<double> & vdVelGearDriveRadS, std::vector<double> & vdAngGearSteerRad)
{
	//LOG_OUT("Calculate Inverse for given Velocity Command");

	CalcInverse();

	vdVelGearDriveRadS = m_vdVelGearDriveTarget1RadS;
	vdAngGearSteerRad = m_vdAngGearSteerTarget1Rad;
}

// Get set point values for the Wheels (including controller) from UndercarriangeCtrl
void UndercarriageCtrlGeom::GetNewCtrlStateSteerDriveSetValues(std::vector<double> & vdVelGearDriveRadS, std::vector<double> & vdVelGearSteerRadS, std::vector<double> & vdAngGearSteerRad,
								 double & dVelLongMMS, double & dVelLatMMS, double & dRotRobRadS, double & dRotVelRadS)
{

	if(m_bEMStopActive == false)
	{
		//Calculate next step
		CalcControlStep();
	}

	vdVelGearDriveRadS = m_vdVelGearDriveCmdRadS;
	vdVelGearSteerRadS = m_vdVelGearSteerCmdRadS;
	vdAngGearSteerRad = m_vdAngGearSteerCmdRad;

	dVelLongMMS = m_dCmdVelLongMMS;
	dVelLatMMS = m_dCmdVelLatMMS;
	dRotRobRadS = m_dCmdRotRobRadS;
	dRotVelRadS = m_dCmdRotVelRadS;

	/*// Logging for debugging		
	// get current time
	m_RawTime.SetNow();
	m_dNowTime = m_RawTime - m_StartTime;
	// Log out resulting joint-commands
	fprintf(m_pfileSteerAngCmd, "%f %f %f %f %f \n", m_dNowTime, m_vdAngGearSteerCmdRad[0], m_vdAngGearSteerCmdRad[1], m_vdAngGearSteerCmdRad[2], m_vdAngGearSteerCmdRad[3]);
	fprintf(m_pfileSteerVelCmd, "%f %f %f %f %f \n", m_dNowTime, m_vdVelGearSteerCmdRadS[0], m_vdVelGearSteerCmdRadS[1], m_vdVelGearSteerCmdRadS[2], m_vdVelGearSteerCmdRadS[3]);
	fprintf(m_pfileDriveVelCmd, "%f %f %f %f %f \n", m_dNowTime, m_vdVelGearDriveCmdRadS[0], m_vdVelGearDriveCmdRadS[1], m_vdVelGearDriveCmdRadS[2], m_vdVelGearDriveCmdRadS[3]);*/
}

// Get result of direct kinematics
void UndercarriageCtrlGeom::GetActualPltfVelocity(double & dDeltaLongMM, double & dDeltaLatMM, double & dDeltaRotRobRad, double & dDeltaRotVelRad,
												  double & dVelLongMMS, double & dVelLatMMS, double & dRotRobRadS, double & dRotVelRadS)
{
	dVelLongMMS = m_dVelLongMMS;
	dVelLatMMS = m_dVelLatMMS;
	dRotRobRadS = m_dRotRobRadS;
	dRotVelRadS = m_dRotVelRadS;

	// calculate travelled distance and angle (from velocity) for output
	// ToDo: make sure this corresponds to cycle-freqnecy of calling node
	//       --> specify via config file
	dDeltaLongMM = dVelLongMMS * m_UnderCarriagePrms.dCmdRateS;
	dDeltaLatMM = dVelLatMMS * m_UnderCarriagePrms.dCmdRateS;
	dDeltaRotRobRad = dRotRobRadS * m_UnderCarriagePrms.dCmdRateS;
	dDeltaRotVelRad = dRotVelRadS * m_UnderCarriagePrms.dCmdRateS;
}

// calculate inverse kinematics
void UndercarriageCtrlGeom::CalcInverse(void)
{	
	// help variable to store velocities of the steering axis in mm/s
	double dtempAxVelXRobMMS, dtempAxVelYRobMMS;	

	// check if zero movement commanded -> keep orientation of wheels, set wheel velocity to zero
	if((m_dCmdVelLongMMS == 0) && (m_dCmdVelLatMMS == 0) && (m_dCmdRotRobRadS == 0) && (m_dCmdRotVelRadS == 0))
	{
		for(int i = 0; i<4; i++)
		{
			m_vdAngGearSteerTarget1Rad[i] = m_vdAngGearSteerRad[i];
			m_vdVelGearDriveTarget1RadS[i] = 0.0;
			m_vdAngGearSteerTarget2Rad[i] = m_vdAngGearSteerRad[i];
			m_vdVelGearDriveTarget2RadS[i] = 0.0;
		}
		return;
	}

	// calculate sets of possible Steering Angle // Drive-Velocity combinations
	for (int i = 0; i<4; i++)
	{	
		// calculate velocity and direction of single wheel motion
		// Translational Portion
		dtempAxVelXRobMMS = m_dCmdVelLongMMS;
		dtempAxVelYRobMMS = m_dCmdVelLatMMS;
		// Rotational Portion
		dtempAxVelXRobMMS += m_dCmdRotRobRadS * m_vdExWheelDistMM[i] * -sin(m_vdExWheelAngRad[i]);
		dtempAxVelYRobMMS += m_dCmdRotRobRadS * m_vdExWheelDistMM[i] * cos(m_vdExWheelAngRad[i]);
		
		// calculate resulting steering angle 
		// Wheel has to move in direction of resulting velocity vector of steering axis 
		m_vdAngGearSteerTarget1Rad[i] = MathSup::atan4quad(dtempAxVelYRobMMS, dtempAxVelXRobMMS);
		// calculate corresponding angle in opposite direction (+180 degree)
		m_vdAngGearSteerTarget2Rad[i] = m_vdAngGearSteerTarget1Rad[i] + MathSup::PI;
		MathSup::normalizePi(m_vdAngGearSteerTarget2Rad[i]);
		
		// calculate absolute value of rotational rate of driving wheels in rad/s
		m_vdVelGearDriveTarget1RadS[i] = sqrt( (dtempAxVelXRobMMS * dtempAxVelXRobMMS) + 
						   (dtempAxVelYRobMMS * dtempAxVelYRobMMS) ) / (double)m_UnderCarriagePrms.iRadiusWheelMM;
		// now adapt to direction (forward/backward) of wheel
		m_vdVelGearDriveTarget2RadS[i] = - m_vdVelGearDriveTarget1RadS[i];
	}
}

// calculate direct kinematics
void UndercarriageCtrlGeom::CalcDirect(void)
{
	// declare auxilliary variables
	double dtempVelXRobMMS;		// Robot-Velocity in x-Direction (longitudinal) in mm/s (in Robot-Coordinateframe)
	double dtempVelYRobMMS;		// Robot-Velocity in y-Direction (lateral) in mm/s (in Robot-Coordinateframe)
	double dtempRotRobRADPS;	// Robot-Rotation-Rate in rad/s (in Robot-Coordinateframe)
	double dtempDiffXMM;		// Difference in X-Coordinate of two wheels in mm
	double dtempDiffYMM;		// Difference in Y-Coordinate of two wheels in mm
	double dtempRelPhiWheelsRAD;	// Angle between axis of two wheels w.r.t the X-Axis of the Robot-Coordinate-System in rad
	double dtempRelDistWheelsMM;	// distance of two wheels in mm 
	double dtempRelPhiWheel1RAD;	// Steering Angle of (im math. pos. direction) first Wheel w.r.t. the linking axis of the two wheels
	double dtempRelPhiWheel2RAD;	// Steering Angle of (im math. pos. direction) first Wheel w.r.t. the linking axis of the two wheels
	std::vector<double> vdtempVelWheelMMS(4);	// Wheel-Velocities (all Wheels) in mm/s

	// initial values
	dtempVelXRobMMS = 0;			// Robot-Velocity in x-Direction (longitudinal) in mm/s (in Robot-Coordinateframe)
	dtempVelYRobMMS = 0;			// Robot-Velocity in y-Direction (lateral) in mm/s (in Robot-Coordinateframe)
	dtempRotRobRADPS = 0;



	// calculate corrected wheel velocities
	for(int i = 0; i<m_iNumberOfDrives; i++)
	{
		// calc effective Driving-Velocity
		vdtempVelWheelMMS[i] = m_UnderCarriagePrms.iRadiusWheelMM * (m_vdVelGearDriveRadS[i] - m_UnderCarriagePrms.vdFactorVel[i]* m_vdVelGearSteerRadS[i]);
	}

	// calculate rotational rate of robot and current "virtual" axis between all wheels
	for(int i = 0; i< (m_iNumberOfDrives-1) ; i++)
	{
		// calc Parameters (Dist,Phi) of virtual linking axis of the two considered wheels
		dtempDiffXMM = m_vdExWheelXPosMM[i+1] - m_vdExWheelXPosMM[i];
		dtempDiffYMM = m_vdExWheelYPosMM[i+1] - m_vdExWheelYPosMM[i];
		dtempRelDistWheelsMM = sqrt( dtempDiffXMM*dtempDiffXMM + dtempDiffYMM*dtempDiffYMM );
		dtempRelPhiWheelsRAD = MathSup::atan4quad( dtempDiffYMM, dtempDiffXMM );

		// transform velocity of wheels into relative coordinate frame of linking axes -> subtract angles
		dtempRelPhiWheel1RAD = m_vdAngGearSteerRad[i] - dtempRelPhiWheelsRAD;
		dtempRelPhiWheel2RAD = m_vdAngGearSteerRad[i+1] - dtempRelPhiWheelsRAD;

		dtempRotRobRADPS += (vdtempVelWheelMMS[i+1] * sin(dtempRelPhiWheel2RAD) - vdtempVelWheelMMS[i] * sin(dtempRelPhiWheel1RAD))/dtempRelDistWheelsMM;
	}

	// calculate last missing axis (between last wheel and 1.)
	// calc. Parameters (Dist,Phi) of virtual linking axis of the two considered wheels
	dtempDiffXMM = m_vdExWheelXPosMM[0] - m_vdExWheelXPosMM[m_iNumberOfDrives-1];
	dtempDiffYMM = m_vdExWheelYPosMM[0] - m_vdExWheelYPosMM[m_iNumberOfDrives-1];
	dtempRelDistWheelsMM = sqrt( dtempDiffXMM*dtempDiffXMM + dtempDiffYMM*dtempDiffYMM );
	dtempRelPhiWheelsRAD = MathSup::atan4quad( dtempDiffYMM, dtempDiffXMM );

	// transform velocity of wheels into relative coordinate frame of linking axes -> subtract angles
	dtempRelPhiWheel1RAD = m_vdAngGearSteerRad[m_iNumberOfDrives-1] - dtempRelPhiWheelsRAD;
	dtempRelPhiWheel2RAD = m_vdAngGearSteerRad[0] - dtempRelPhiWheelsRAD;

	// close calculation of robots rotational velocity
	dtempRotRobRADPS += (vdtempVelWheelMMS[0]*sin(dtempRelPhiWheel2RAD) - vdtempVelWheelMMS[m_iNumberOfDrives-1]*sin(dtempRelPhiWheel1RAD))/dtempRelDistWheelsMM;

	// calculate linear velocity of robot
	for(int i = 0; i<m_iNumberOfDrives; i++)
	{
		dtempVelXRobMMS += vdtempVelWheelMMS[i]*cos(m_vdAngGearSteerRad[i]);
		dtempVelYRobMMS += vdtempVelWheelMMS[i]*sin(m_vdAngGearSteerRad[i]);
	}

	// assign rotational velocities for output
	m_dRotRobRadS = dtempRotRobRADPS/m_iNumberOfDrives;
	m_dRotVelRadS = 0; // currently not used to represent 3rd degree of freedom -> set to zero

	// assign linear velocity of robot for output
	m_dVelLongMMS = dtempVelXRobMMS/m_iNumberOfDrives;
	m_dVelLatMMS = dtempVelYRobMMS/m_iNumberOfDrives;

	

}

// calculate Exact Wheel Position in robot coordinates
void UndercarriageCtrlGeom::CalcExWheelPos(void)
{
	// calculate wheel position and velocity
	for(int i = 0; i<4; i++)
	{
		// calculate current geometry of robot (exact wheel position, taking into account steering offset of wheels)
		m_vdExWheelXPosMM[i] = m_vdWheelXPosMM[i] + m_UnderCarriagePrms.iDistSteerAxisToDriveWheelMM * sin(m_vdAngGearSteerRad[i]);
		m_vdExWheelYPosMM[i] = m_vdWheelYPosMM[i] - m_UnderCarriagePrms.iDistSteerAxisToDriveWheelMM * cos(m_vdAngGearSteerRad[i]);
				
		// calculate distance from platform center to wheel center
		m_vdExWheelDistMM[i] = sqrt( (m_vdExWheelXPosMM[i] * m_vdExWheelXPosMM[i]) + (m_vdExWheelYPosMM[i] * m_vdExWheelYPosMM[i]) );
		
		// calculate direction of rotational vector
		m_vdExWheelAngRad[i] = MathSup::atan4quad( m_vdExWheelYPosMM[i], m_vdExWheelXPosMM[i]);
	}
}

// perform one discrete Control Step (controls steering angle)
void UndercarriageCtrlGeom::CalcControlStep(void)
{	
	// check if zero movement commanded -> keep orientation of wheels, set steer velocity to zero
	if ((m_dCmdVelLongMMS == 0) && (m_dCmdVelLatMMS == 0) && (m_dCmdRotRobRadS == 0) && (m_dCmdRotVelRadS == 0))
	{
		m_vdVelGearDriveCmdRadS.assign(4,0.0);		// set velocity for drives to zero
		m_vdVelGearSteerCmdRadS.assign(4,0.0);		// set velocity for steers to zero

		// set internal states of controller to zero
		for(int i=0; i<4; i++)
		{
			m_vdCtrlVal[i][0] = 0.0;
			m_vdCtrlVal[i][1] = 0.0;
		}
		return;
	}

	// declare auxilliary variables
	double dCurrentPosWheelRAD;
	double dDeltaPhi;
	double dForceDamp, dForceProp, dAccCmd, dVelCmdInt; // PI- and Impedance-Ctrl
	
	for (int i=0; i<4; i++)
	{
		// provisorial --> skip interpolation and always take Target
		m_vdVelGearDriveCmdRadS[i] = m_vdVelGearDriveTargetRadS[i];
		m_vdAngGearSteerCmdRad[i] = m_vdAngGearSteerTargetRad[i];

		// provisorial --> skip interpolation and always take Target1
		//m_vdVelGearDriveCmdRadS[i] = m_vdVelGearDriveTarget1RadS[i];
		//m_vdAngGearSteerCmdRad[i] = m_vdAngGearSteerTarget1Rad[i];

		/*m_vdAngGearSteerIntpRad[i] += m_vdDeltaAngIntpRad[i];
		MathSup::normalizePi(m_vdAngGearSteerIntpRad[i]);
		m_vdVelGearDriveIntpRadS[i] += m_vdDeltaDriveIntpRadS[i];

		m_vdVelGearDriveCmdRadS[i] = m_vdVelGearDriveIntpRadS[i];
		m_vdAngGearSteerCmdRad[i] = m_vdAngGearSteerIntpRad[i];*/
	}


	for (int i = 0; i<4; i++)
	{
		// Normalize Actual Wheel Position before calculation
		dCurrentPosWheelRAD = m_vdAngGearSteerRad[i];
		MathSup::normalizePi(dCurrentPosWheelRAD);
		dDeltaPhi = m_vdAngGearSteerCmdRad[i] - dCurrentPosWheelRAD;
		MathSup::normalizePi(dDeltaPhi);

		// Impedance-Ctrl
		// Calculate resulting desired forces, velocities
		// double dForceDamp, dForceProp, dAccCmd, dVelCmdInt;
		dForceDamp = - m_dDamp * m_vdCtrlVal[i][1];
		dForceProp = m_dSpring * dDeltaPhi;
		dAccCmd = (dForceDamp + dForceProp) / m_dVirtM;
		if (dAccCmd > m_dDDPhiMax)
		{
			dAccCmd = m_dDDPhiMax;
		}
		else if (dAccCmd < -m_dDDPhiMax)
		{
			dAccCmd = -m_dDDPhiMax;
		}
		dVelCmdInt = m_vdCtrlVal[i][1] + m_UnderCarriagePrms.dCmdRateS * dAccCmd;
		if (dVelCmdInt > m_dDPhiMax)
		{
			dVelCmdInt = m_dDPhiMax;
		}
		else if (dVelCmdInt < -m_dDPhiMax)
		{
			dVelCmdInt = -m_dDPhiMax;
		}
		// Store internal ctrlr-states
		m_vdCtrlVal[i][0] = dDeltaPhi;
		m_vdCtrlVal[i][1] = dVelCmdInt;
		// set outputs
		m_vdVelGearSteerCmdRadS[i] = dVelCmdInt;

		// Check if Steeringvelocity overgo maximum allowed rates.
		if(fabs(m_vdVelGearSteerCmdRadS[i]) > m_UnderCarriagePrms.dMaxSteerRateRadpS)
		{
			if (m_vdVelGearSteerCmdRadS[i] > 0)
				m_vdVelGearSteerCmdRadS[i] = m_UnderCarriagePrms.dMaxSteerRateRadpS;
			else
				m_vdVelGearSteerCmdRadS[i] = -m_UnderCarriagePrms.dMaxSteerRateRadpS;
		}
	}
	
	// Correct Driving-Wheel-Velocity, because of coupling and axis-offset
	for (int i = 0; i<4; i++)
	{
		m_vdVelGearDriveCmdRadS[i] += m_vdVelGearSteerCmdRadS[i] * m_UnderCarriagePrms.vdFactorVel[i];
	}

}

// operator overloading
void UndercarriageCtrlGeom::operator=(const UndercarriageCtrlGeom & GeomCtrl)
{
	// Actual Values for PltfMovement (calculated from Actual Wheelspeeds)
	m_dVelLongMMS = GeomCtrl.m_dVelLongMMS;
	m_dVelLatMMS = GeomCtrl.m_dVelLatMMS;
	m_dRotRobRadS = GeomCtrl.m_dRotRobRadS;
	m_dRotVelRadS = GeomCtrl.m_dRotVelRadS;

	// Actual Wheelspeed (read from Motor-Ctrls)
	m_vdVelGearDriveRadS = GeomCtrl.m_vdVelGearDriveRadS;
	m_vdVelGearSteerRadS = GeomCtrl.m_vdVelGearSteerRadS;
	m_vdDltAngGearDriveRad = GeomCtrl.m_vdDltAngGearDriveRad;
	m_vdAngGearSteerRad = GeomCtrl.m_vdAngGearSteerRad;

	// Desired Pltf-Movement (set from PltfHwItf)
	m_dCmdVelLongMMS = GeomCtrl.m_dCmdVelLongMMS;
	m_dCmdVelLatMMS = GeomCtrl.m_dCmdVelLatMMS;
	m_dCmdRotRobRadS = GeomCtrl.m_dCmdRotRobRadS;
	m_dCmdRotVelRadS = GeomCtrl.m_dCmdRotVelRadS;

	// Desired Wheelspeeds (calculated from desired ICM-configuration)
	m_vdVelGearDriveCmdRadS = GeomCtrl.m_vdVelGearDriveCmdRadS;
	m_vdVelGearSteerCmdRadS = GeomCtrl.m_vdVelGearSteerCmdRadS;
	m_vdAngGearSteerCmdRad = GeomCtrl.m_vdAngGearSteerCmdRad;

	// Target Wheelspeed and -angle (calculated from desired Pltf-Movement with Inverse without controle!)
	// alternativ 1 for steering angle
	m_vdAngGearSteerTarget1Rad = GeomCtrl.m_vdAngGearSteerTarget1Rad;
	m_vdVelGearDriveTarget1RadS = GeomCtrl.m_vdVelGearDriveTarget1RadS;
	// alternativ 2 for steering angle (+/- PI)
	m_vdAngGearSteerTarget2Rad = GeomCtrl.m_vdAngGearSteerTarget2Rad;
	m_vdVelGearDriveTarget2RadS = GeomCtrl.m_vdVelGearDriveTarget2RadS;

	// Position of the Wheels' Steering Axis'
	m_vdWheelXPosMM = GeomCtrl.m_vdWheelXPosMM;
	m_vdWheelYPosMM = GeomCtrl.m_vdWheelYPosMM;
	m_vdWheelDistMM = GeomCtrl.m_vdWheelDistMM;
	m_vdWheelAngRad = GeomCtrl.m_vdWheelAngRad;

	// Exact Position of the Wheels' itself
	m_vdExWheelXPosMM = GeomCtrl.m_vdExWheelXPosMM;
	m_vdExWheelYPosMM = GeomCtrl.m_vdExWheelYPosMM;
	m_vdExWheelDistMM = GeomCtrl.m_vdExWheelDistMM;
	m_vdExWheelAngRad = GeomCtrl.m_vdExWheelAngRad;

	// Prms
	m_UnderCarriagePrms = GeomCtrl.m_UnderCarriagePrms;

	// Position Controller Steer Wheels
	// Impedance-Ctrlr
	m_dSpring = GeomCtrl.m_dSpring;
	m_dDamp = GeomCtrl.m_dDamp;
	m_dVirtM = GeomCtrl.m_dDPhiMax;
	m_dDPhiMax = GeomCtrl.m_dDPhiMax;
	m_dDDPhiMax = GeomCtrl.m_dDDPhiMax;
	// Storage for internal controller states
	m_vdCtrlVal = GeomCtrl.m_vdCtrlVal;
}

// set EM Flag and stop ctrlr if active
void UndercarriageCtrlGeom::setEMStopActive(bool bEMStopActive)
{
	m_bEMStopActive = bEMStopActive;

	// if emergency stop reset ctrlr to zero
	if(m_bEMStopActive)
	{
		// Steermodules
		for(int i=0; i<4; i++)
		{
			for(int j=0; j< 2; j++)
			{
				m_vdCtrlVal[i][j] = 0.0;
			}
		}
		// Outputs
		for(int i=0; i<4; i++)
		{
			m_vdVelGearDriveCmdRadS[i] = 0.0;
			m_vdVelGearSteerCmdRadS[i] = 0.0;
		}
	}

}
