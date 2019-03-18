/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#include "PoseController.h"
#include <MathLib/Quaternion.h>
#include "ConUtils.h"
#include <Utils/Utils.h>

PoseController::PoseController(Character* ch) : Controller(ch){

	//copy the current state of the character into the desired pose - makes sure that it's the correct size
	ch->getState(&desiredPose);

	//set initial values
	ReducedCharacterState rs(&desiredPose);
	rs.setPosition(Vector3d());
	rs.setVelocity(Vector3d());
	rs.setOrientation(Quaternion(1, 0, 0, 0));
	rs.setAngularVelocity(Vector3d());

	for (int i=0;i<jointCount;i++){
		controlParams.push_back(ControlParams());
		rs.setJointRelativeAngVelocity(Vector3d(), i);
		rs.setJointRelativeOrientation(Quaternion(), i);
		torques.push_back(Vector3d());
	}
}

PoseController::~PoseController(void){
}

/**
	This method is used to compute the PD torque that aligns a child coordinate frame to a parent coordinate frame.
	Given: the current relative orientation of two coordinate frames (child and parent), the relative angular velocity,
	the desired values for the relative orientation and ang. vel, as well as the virtual motor's PD gains. The torque 
	returned is expressed in the coordinate frame of the 'parent'.
*/
Vector3d PoseController::computePDTorque(const Quaternion& qRel, const Quaternion& qRelD, const Vector3d& wRel, const Vector3d& wRelD, ControlParams* cParams){
	Vector3d torque;
	//the torque will have the form:
	// T = kp*D(qRelD, qRel) + kd * (wRelD - wRel)

	//Note: There can be problems computing the proper torque from the quaternion part, because q and -q 
	//represent the same orientation. To make sure that we get the correct answer, we'll take into account
	//the sign of the scalar part of qErr - both this and the v part will change signs in the same way if either 
	//or both of qRel and qRelD are negative

//	Quaternion qErr = qRel.getComplexConjugate() * qRelD;
	Quaternion qErr = qRel.getComplexConjugate();
	qErr *= qRelD;

	//qErr.v also contains information regarding the axis of rotation and the angle (sin(theta)), but I want to scale it by theta instead
	double sinTheta = qErr.v.length();
	if (sinTheta>1)
		sinTheta = 1;
	if (IS_ZERO(sinTheta)){
		//avoid the divide by close-to-zero. The orientations match, so the proportional component of the torque should be 0
	}else{
		double absAngle = 2 * asin(sinTheta);
		torque = qErr.v;
		torque *= 1/sinTheta * absAngle * (-cParams->kp) * SGN(qErr.s);
//		torque = qErr.v/sinTheta * absAngle * (-cParams->kp) * SGN(qErr.s);
	}

	//qErr represents the rotation from the desired child frame to the actual child frame, which
	//means that the torque is now expressed in child coordinates. We need to express it in parent coordinates!
	torque = qRel.rotate(torque);
	//the angular velocities are stored in parent coordinates, so it is ok to add this term now
	torque += (wRelD - wRel) * (-cParams->kd);
	torque *= cParams->strength;

	//now the torque is stored in parent coordinates - we need to scale it and apply torque limits
	scaleAndLimitTorque(&torque, cParams, qRel.getComplexConjugate());

	//and we're done...
	return torque;
}

/**
	This method is used to apply joint limits to the torque passed in as a parameter. It is assumed that
	the torque is already represented in the correct coordinate frame
*/
void PoseController::limitTorque(Vector3d* torque, ControlParams* cParams){
	if (torque->x < -cParams->scale.x * cParams->maxAbsTorque) torque->x = -cParams->scale.x * cParams->maxAbsTorque;
	if (torque->x > cParams->scale.x * cParams->maxAbsTorque) torque->x = cParams->scale.x * cParams->maxAbsTorque;
	if (torque->y < -cParams->scale.y * cParams->maxAbsTorque) torque->y = -cParams->scale.y * cParams->maxAbsTorque;
	if (torque->y > cParams->scale.y * cParams->maxAbsTorque) torque->y = cParams->scale.y * cParams->maxAbsTorque;
	if (torque->z < -cParams->scale.z * cParams->maxAbsTorque) torque->z = -cParams->scale.z * cParams->maxAbsTorque;
	if (torque->z > cParams->scale.z * cParams->maxAbsTorque) torque->z = cParams->scale.z * cParams->maxAbsTorque;
}

/**
	This method is used to scale and apply joint limits to the torque that is passed in as a parameter. The orientation that transforms 
	the torque from the coordinate frame that it is currently stored in, to the coordinate frame of the 'child' to which the torque is 
	applied to (it wouldn't make sense to scale the torques in any other coordinate frame)  is also passed in as a parameter.
*/
void PoseController::scaleAndLimitTorque(Vector3d* torque, ControlParams* cParams, const Quaternion& qToChild){
	//now change the torque to child coordinates
	*torque = qToChild.rotate(*torque);

	//and scale it differently along the main axis...
	torque->x *= cParams->scale.x;
	torque->y *= cParams->scale.y;
	torque->z *= cParams->scale.z;

	limitTorque(torque, cParams);

	// and now change it back to the original coordinates
	*torque = qToChild.getComplexConjugate().rotate(*torque);
}

// --Reimplemented by Min and Zhiyi--
void PoseController::newComputeTorques(DynamicArray<ContactPoint> *cfs)
{
    Quaternion relQ; // relative orientation
    Vector3d relW; // relative angular velocity

    Quaternion targetQ; // target orientation
    Vector3d targetW;  // target angular velocity

    ReducedCharacterState rs(&desiredPose);

    Quaternion qError;

    float errAngle; //dTheta - Theta, where Theta is current angle, dTheta is target angle set by current state in the FSM
    int sign;

    // Since Simbicon internally uses Quaternion to represent joints' relative orientations,
    // to calculate the difference between current angle and desired angle, we need to do some
    // conversion from Quaternion back to absolute angle.

    // Refer to the paper ->
    // T = kp * (dTheta - Theta) + kd * (dW - w)
    for (int i = 0; i < jointCount; i++)
    {
        targetQ = rs.getJointRelativeOrientation(i);
        targetW = rs.getJointRelativeAngVelocity(i);

        if (controlParams[i].controlled == true)
        {

            if (controlParams[i].relToCharFrame == false)
            {
                character->getRelativeOrientation(i, &relQ);
                character->getRelativeAngularVelocity(i, &relW);
            }

            else
            {
                RigidBody* child = character->getJoint(i)->getChild();
                relQ = child->getOrientation();
                relW = child->getAngularVelocity();

                targetQ *= controlParams[i].charFrame;
            }

            // Refer to the paper ->
            // T = kp * (dTheta - Theta) + kd * (dW - w)
            // kp -> proportional gain
            // kd -> derivative gain
            // Rotations are represented in quarternion
            // composition of rotations using quaternion:
            // q(Theta) * q(dTheta - Theta)= q(dTheta) ->
            // q(dTheta - Theta) = inverse(q(Theta)) * q(dTheta) ->
            // q(dTheta - Theta) = q(Theta).conjugate * q(dTheta), since q(Theta) is a unit quaternion
            qError = relQ.getComplexConjugate() * targetQ;

            //sin(dTheta - Theta / 2) = qError.v.length()
            float sinErr = qError.v.length();

            if (sinErr > 1)
                sinErr = 1;

            // if sin(dTheta - Theta / 2) roughly equals  to 0 ->
            // dTheta = Theta, no need to use proportional term
            if (sinErr >= -0.0000000001 && sinErr <= -0.0000000001)
            {
                //nothing need to be done here
            }

            else
            {
                errAngle = 2 * asin(sinErr); // errAngle = dTheta - Theta,
                torques[i] = qError.v / sinErr; // get the vector part of the quaternion to get the rotation axis for derived torque


                // This part handles the sign change
                // as the original implementation pointed out in the comment session
                // q and -q represent the same orientation, use q(dTheta - Theta) to
                // handle the situation where q(dTheta) and  q(Theta) have different sign
                if (qError.s < 0)
                    sign = -1;

                else
                    sign = 1;

                //kp * (dTheta - Theta)
                torques[i] *= errAngle * (-controlParams[i].kp) * sign;

            } // apply the proportional term


            // transform the torque from child coordinates to parent frame
            torques[i] = targetQ.rotate(torques[i]);

            // apply the derivative term
            torques[i] += (targetW - relW) * (-controlParams[i].kd);

            // apply the force
            torques[i] *= controlParams[i].strength;

            // now the torque is stored in parent coordinates - we need to scale it and apply torque limits
            scaleAndLimitTorque(&torques[i], &controlParams[i], relQ.getComplexConjugate());

            // express the torque in world coordinates
            torques[i] = character->getJoint(i)->getParent()->getWorldCoordinates(torques[i]);

        }

        else 
            torques[i].setValues(0, 0, 0);
    }
}

/**
	This method is used to parse the information passed in the string. This class knows how to read lines
	that have the name of a joint, followed by a list of the pertinent parameters. If this assumption is not held,
	then classes extended this one are required to provide their own implementation of this simple parser
*/
void PoseController::parseGainLine(char* line){
	double kp, kd, tMax, scX, scY, scZ;
	char jName[100];
	int jIndex;
	int nrParams = 0;
	nrParams = sscanf(line, "%s %lf %lf %lf %lf %lf %lf\n", jName, &kp, &kd, &tMax, &scX, &scY, &scZ); 
	if (nrParams == 2){
		Vector3d tmp;
		character->getJointByName(jName)->getChild()->props.getLocalMOI(&tmp);
		double maxM = max(tmp.x, tmp.y); maxM = max(maxM, tmp.z);
		kd = kp/10;
		tMax = 10000;
		scX = tmp.x/maxM;
		scY = tmp.y/maxM;
		scZ = tmp.z/maxM;
	}else
	if (nrParams!=7)
		throwError("To specify the gains, you need: 'joint name Kp Kd Tmax scaleX scaleY scaleZ'! --> \'%s\'", line);
	jIndex = character->getJointIndex(jName);
	if (jIndex < 0)
		throwError("Cannot find joint: \'%s\'", jName);
	controlParams[jIndex].kp = kp;
	controlParams[jIndex].kd = kd;
	controlParams[jIndex].maxAbsTorque = tMax;
	controlParams[jIndex].scale = Vector3d(scX, scY, scZ);
}


/**
	This method is used to read the gain coefficients, as well as max torque allowed for each joint
	from the file that is passed in as a parameter.
*/
void PoseController::readGains(FILE* f){
	if (f == NULL)
		throwError("File pointer is NULL - cannot read gain coefficients!!");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];

	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer)>195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		int lineType = getConLineType(line);
		switch (lineType) {
			case CON_PD_GAINS_END:
				return;
				break;
			case CON_PD_GAINS_START:
				break;
			case CON_COMMENT:
				break;
			default:
				parseGainLine(line);
			break;
		}
	}
	throwError("Incorrect controller input file: No \'/KpKdMaxT\' found", buffer);
}



/**
	This method is used to write the gain coefficients, as well as max torque allowed for each joint
	from the file that is passed in as a parameter.
*/
void PoseController::writeGains(FILE* f){

	for( uint jIndex=0; jIndex < controlParams.size(); ++jIndex ) {

		Joint* joint = character->getJoint(jIndex);
		if( !joint ) continue;
		const char* jName = joint->getName();

		fprintf( f, "    %s\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", 
						jName, 
						controlParams[jIndex].kp,
						controlParams[jIndex].kd,
						controlParams[jIndex].maxAbsTorque,
						controlParams[jIndex].scale.x,
						controlParams[jIndex].scale.y,
						controlParams[jIndex].scale.z );

	}

}


/**
	This method is used to read the gain coefficients, as well as max torque allowed for each joint
	from the file that is passed in as a parameter.
*/
void PoseController::readGains(char* fName){
	FILE* f = fopen(fName, "r");
	if (f == NULL)
		return;

	readGains(f);
	fclose(f);
}

/**
	sets the targets to match the current state of the character
*/
void PoseController::setTargetsFromState(){
	ReducedCharacterState rs(&desiredPose);
	Quaternion qTemp;
	for (int i=0;i<jointCount;i++){
		character->getRelativeOrientation(i, &qTemp);
		rs.setJointRelativeOrientation(qTemp, i);
	}
}

