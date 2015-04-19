/* Copyright (C) 2015  Matteo Hausner
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"

#include <math.h>
#include <string.h>

// define name
#define NAME "Hughes 500D"
#define NAME_LOWERCASE "hughes_500d"

// define version
#define VERSION "0.1"

// define constants
#define MAX_DOOR_SPEED 0.8f
#define MAX_ROTATION 720.0f
#define HEAD_ROTATION_SPEED 150.0f

// global dataref variables
static XPLMDataRef doorsLeftPositionDataRef = NULL, doorsRightPositionDataRef = NULL, adf1DataRef = NULL, adf2DataRef = NULL, com1DataRef = NULL, com2DataRef = NULL, dmeDataRef = NULL, nav1DataRef = NULL, nav2DataRef = NULL, tacradsHighMainDataRef = NULL, tacradsHighTailDataRef = NULL, headHeadingDataRef = NULL, rotorBladesPitch0DataRef = NULL, rotorBladesPitch1DataRef = NULL, rotorBladesPitch2DataRef = NULL, rotorBladesPitch3DataRef = NULL, rotorBladesPitch4DataRef = NULL, rotorMutingLowPitchDataRef = NULL, rotorMutingLowRollDataRef = NULL, rotorPositionMainDataRef = NULL, rotorPositionMainMutingDataRef = NULL, rotorPositionTailDataRef = NULL, rotorPositionTailMutingDataRef = NULL, rotorPositionMainFpsMutingDataRef = NULL, rotorPositionTailFpsMutingDataRef = NULL, acfNumBladesDataRef = NULL, acfCyclicAilnDataRef = NULL, acfCyclicElevDataRef = NULL, audioPanelOutDataRef = NULL, flaprqstDataRef = NULL, cyclicElevDiscTiltDataRef = NULL, cyclicAilnDiscTiltDataRef = NULL, pointPitchDegDataRef = NULL, pointTacradDataRef = NULL, ongroundAnyDataRef = NULL, localXDataRef = NULL, localZDataRef = NULL, phiDataRef = NULL, psiDataRef = NULL, pDotDataRef = NULL, qDotDataRef = NULL, viewXDataRef = NULL, viewZDataRef = NULL, yolkPitchRatioDataRef = NULL, yolkRollRatioDataRef = NULL, frameRatePeriodDataRef = NULL;

// global internal variables
int doorBounce = 0;
float doorSpeed = 0.0f, doorsLeftPosition= 0.0f, doorsRightPosition= 0.0f, adf1= 0.0f, adf2= 0.0f, com1= 0.0f, com2= 0.0f, dme= 0.0f, nav1= 0.0f, nav2= 0.0f, tacradsHighMain= 0.0f, tacradsHighTail= 0.0f, headHeading= 0.0f, rotorBladesPitch0= 0.0f, rotorBladesPitch1= 0.0f, rotorBladesPitch2= 0.0f, rotorBladesPitch3= 0.0f, rotorBladesPitch4= 0.0f, rotorMutingLowPitch= 0.0f, rotorMutingLowRoll= 0.0f, rotorPositionMain= 0.0f, rotorPositionMainMuting= 0.0f, rotorPositionTail= 0.0f, rotorPositionTailMuting= 0.0f, rotorPositionMainFpsMuting= 0.0f, rotorPositionTailFpsMuting= 0.0f;

static void UpdateDoors(void)
{
    float frameRatePeriod = XPLMGetDataf(frameRatePeriodDataRef);

    if(XPLMGetDataf(flaprqstDataRef) > 0.0f)
    // doors open
    {
        if(doorsLeftPosition < 1.0f && doorBounce == 0)
        {
            doorBounce = 0;
					  doorSpeed = MAX_DOOR_SPEED * (1.5f - doorsLeftPosition);

					  float newDoorPosition = doorsLeftPosition + doorSpeed * frameRatePeriod;

					  if (newDoorPosition > 1.0f)
                newDoorPosition = 1.0f;

            doorsLeftPosition = newDoorPosition;
            doorsRightPosition = newDoorPosition;
        }
				else
        {
					  doorBounce = 1;
					  doorSpeed = MAX_DOOR_SPEED * (doorsLeftPosition - 0.87f);

            if (doorSpeed < 0.01f)
                doorSpeed = 0.0f;

            if (doorSpeed > 0.0f)
            {
                float newDoorPosition = doorsLeftPosition - doorSpeed * frameRatePeriod;
						
                if (newDoorPosition < 0.0f)
                {
							      newDoorPosition = 0.0f;
							      doorBounce = 0;
                }

                doorsLeftPosition = newDoorPosition;
                doorsRightPosition = newDoorPosition;
            }
		    }
    }
    else
    // doors closed
    {
        doorBounce = 0;
				doorSpeed = MAX_DOOR_SPEED * (1.2f - doorsLeftPosition);

				if (doorsLeftPosition > 0.0f)
        {
            float newDoorPosition = doorsLeftPosition - doorSpeed * frameRatePeriod;
            if (newDoorPosition < 0.0f)
						    newDoorPosition = 0.0f;

            doorsLeftPosition = newDoorPosition;
            doorsRightPosition = newDoorPosition;
				}
    }
}

// converts from degrees to radians
inline static double RadiansToDegress(double radians)
{
    return radians * (180.0 / M_PI);
}

// converts from degrees to radians
inline static double DegreesToRadians(double degrees)
{
    return degrees * (M_PI / 180.0);
}

static void UpdateRotor(void)
{
    float pointTacrad[8];
    XPLMGetDatavf(pointTacradDataRef, pointTacrad, 0, 8);
    
    float frameRatePeriod = XPLMGetDataf(frameRatePeriodDataRef);
    
    // main rotor
    float v1 = XPLMGetDataf(rotorPositionMainDataRef) + RadiansToDegress(pointTacrad[0]) * frameRatePeriod;
    if (v1 > MAX_ROTATION )
        v1 -= MAX_ROTATION;
    else if (v1 < -MAX_ROTATION)
        v1 += MAX_ROTATION;
    rotorPositionMain = v1;

    // tail rotor
    float v2 = XPLMGetDataf(rotorPositionTailDataRef) + RadiansToDegress(pointTacrad[1]) * frameRatePeriod;
    if (v2 > MAX_ROTATION )
        v2 -= MAX_ROTATION;
    else if (v2 < -MAX_ROTATION)
        v2 += MAX_ROTATION;
	  rotorPositionTail = v2;
    
    float cyclicElevDiscTilt = 0.0f;
    XPLMGetDatavf(cyclicElevDiscTiltDataRef, &cyclicElevDiscTilt, 0, 1);
    float cyclicAilnDiscTilt = 0.0f;
    XPLMGetDatavf(cyclicAilnDiscTiltDataRef, &cyclicAilnDiscTilt, 0, 1);

    float newCyclicElevDiscTilt = 0.0f;
    float newCyclicAilnDiscTilt = 0.0f;
    float newRotorMutingLowPitch = 0.0f;
    float newRotorMutingLowRoll = 0.0f;
    
    if (pointTacrad[0] >= 15.0f)
    {
        tacradsHighMain = 1.0f;
        // TODO: XPLMSetDataf(xcdr_rotorPositionDegressMainMuted, 0.0f);

        // low speed rotor
        newCyclicElevDiscTilt = 0.0f;
        newCyclicAilnDiscTilt = 0.0f;

        // high speed rotor
        newRotorMutingLowPitch = cyclicElevDiscTilt;
        newRotorMutingLowRoll = cyclicAilnDiscTilt;

        // fps based accumulators
        float fpsAccMain = XPLMGetDataf(rotorPositionMainFpsMutingDataRef);
        if (fpsAccMain > 36000.0f)
            fpsAccMain -= 36000.0f;
          
        rotorPositionMainFpsMuting = fpsAccMain + 36.0f;
        rotorPositionMainMuting = 0.0f;
    }
    else
    {
        tacradsHighMain = 0.0f;
        rotorPositionMainFpsMuting = v1;

        // low speed rotor
        newCyclicElevDiscTilt = cyclicElevDiscTilt;
        newCyclicAilnDiscTilt = cyclicAilnDiscTilt;

        // high speed rotor
        newRotorMutingLowPitch = 0.0f;
        newRotorMutingLowRoll = 0.0f;

        rotorPositionMainFpsMuting = 0.0f;
    }
    
    XPLMSetDataf(cyclicElevDiscTiltDataRef, newCyclicElevDiscTilt);
    XPLMSetDataf(cyclicAilnDiscTiltDataRef, newCyclicAilnDiscTilt);
    rotorMutingLowPitch = newRotorMutingLowPitch;
    rotorMutingLowRoll = newRotorMutingLowRoll;
    
    if (pointTacrad[1] >= 15.0f)
    {
        tacradsHighTail = 1.0f;
        rotorPositionTailMuting = 0.0f;

        // fps based accumulators
        float fpsAccTail = rotorPositionTailFpsMuting;
        if( fpsAccTail > 36000.0f)
            fpsAccTail -= 36000.0f;
        rotorPositionTailFpsMuting = fpsAccTail + 36.0f;
    }
    else
    {
        tacradsHighTail = 0.0f;
        rotorPositionTailMuting = v2;
        rotorPositionTailFpsMuting = 0.0f;
    }
    
    float acfNumBlades = 0.0f;
    XPLMGetDatavf(acfNumBladesDataRef, &acfNumBlades, 0, 1);
    if (acfNumBlades > 5.0f)
        acfNumBlades = 5.0f;

    float bladeOffsetStep = 360.0f / acfNumBlades;
    float propAngle = XPLMGetDataf(rotorPositionMainDataRef) - bladeOffsetStep * 0.5f;
    
    float pointPitchDeg = 0.0f;
    XPLMGetDatavf(pointPitchDegDataRef, &pointPitchDeg, 0, 1);

    float bladePitch[5];
    for (int i = 0; i < 5; i++)
    {
        float bladeOffset = DegreesToRadians(propAngle + i * bladeOffsetStep);

        bladePitch[i] = (((XPLMGetDataf(acfCyclicAilnDataRef) * XPLMGetDataf(yolkRollRatioDataRef) * cos(bladeOffset)) - (XPLMGetDataf(acfCyclicElevDataRef) * XPLMGetDataf( yolkPitchRatioDataRef) * sin(bladeOffset))) * -1.0f) + pointPitchDeg;
    }

    rotorBladesPitch0 = bladePitch[0];
    rotorBladesPitch1 = bladePitch[1];
    rotorBladesPitch2 = bladePitch[2];
    rotorBladesPitch3 = bladePitch[3];
    rotorBladesPitch4 = bladePitch[4];
}

inline static float CourseToLocation(float deltaX, float deltaY)
{
    return atan2(deltaY, deltaX) * 180.0f / M_PI;
}

static void UpdatePilot(void)
{
    float headHeading = XPLMGetDataf(headHeadingDataRef);
    float targetHeading = 0.0f;
    
    if (XPLMGetDatai(ongroundAnyDataRef) == 1)
    // aircraft on ground
    {
        float targetHeading = CourseToLocation(XPLMGetDataf(viewXDataRef) - XPLMGetDataf(localXDataRef), XPLMGetDataf(viewZDataRef) - XPLMGetDataf(localZDataRef)) - XPLMGetDataf(psiDataRef);

        if (targetHeading > 180.0f)
            targetHeading -= 360.0f;
        else if (targetHeading < -180.0f)
            targetHeading += 360.0f;

        if (targetHeading > 92.0f || targetHeading < -100.0f)
            targetHeading = 0.0f;
    }
    // aircraft not on ground
    else
        targetHeading = XPLMGetDataf(phiDataRef);

    if (targetHeading < -70.0f)
        targetHeading = -70.0f;
    else if (targetHeading > 70.0f)
        targetHeading = 70.0f;
      
    float headingTargetDistancePercent = (targetHeading - headHeading) / 25.0f;
    
    if (headingTargetDistancePercent > 1.0f)
        headingTargetDistancePercent = 1.0f;
    else if (headingTargetDistancePercent < -1.0f)
        headingTargetDistancePercent = -1.0f;
        
    float headRotationSpeed = 150.0f;

    headHeading += HEAD_ROTATION_SPEED * headingTargetDistancePercent * XPLMGetDataf(frameRatePeriodDataRef);
    
    if (headHeading < -70.0f)
          headHeading = -70.0f;
    else if (headHeading > 70.0f)
          headHeading = 70.0f;

    headHeading = headHeading;
}

static void UpdateSwitches(void)
{
    switch (XPLMGetDatai(audioPanelOutDataRef))
    {
        case 0:
            adf1 = 0.0f;
            adf2 = 0.0f;
            com1 = 0.0f;
            com2 = 0.0f;
            dme = 0.0f;
            nav1 = 1.0f;
            nav2 = 0.0f;
            break;
            
        case 1:
            adf1 = 0.0f;
            adf2 = 0.0f;
            com1 = 0.0f;
            com2 = 0.0f;
            dme = 0.0f;
            nav1 = 0.0f;
            nav2 = 1.0f;
            break;
            
        case 2:
            adf1 = 1.0f;
            adf2 = 0.0f;
            com1 = 0.0f;
            com2 = 0.0f;
            dme = 0.0f;
            nav1 = 0.0f;
            nav2 = 0.0f;
            break;
            
        case 3:
            adf1 = 0.0f;
            adf2 = 1.0f;
            com1 = 0.0f;
            com2 = 0.0f;
            dme = 0.0f;
            nav1 = 0.0f;
            nav2 = 0.0f;
            break;
            
        case 5:
            adf1 = 0.0f;
            adf2 = 0.0f;
            com1 = 0.0f;
            com2 = 0.0f;
            dme = 1.0f;
            nav1 = 0.0f;
            nav2 = 0.0f;
            break;
            
        case 10:
            adf1 = 0.0f;
            adf2 = 0.0f;
            com1 = 1.0f;
            com2 = 0.0f;
            dme = 0.0f;
            nav1 = 0.0f;
            nav2 = 0.0f;
            break;
            
        case 11:
            adf1 = 0.0f;
            adf2 = 0.0f;
            com1 = 0.0f;
            com2 = 1.0f;
            dme = 0.0f;
            nav1 = 0.0f;
            nav2 = 0.0f;
            break;
    }
}

static void UpdateTransitionalShudder(void)
{
    float p = XPLMGetDataf(pDotDataRef);
		float q = XPLMGetDataf(qDotDataRef);

    if (XPLMGetDatai(ongroundAnyDataRef))
    {
		    p *= 0.001f;
				q *= 0.5f;
    }
    
    float pointTacrad[8];
    XPLMGetDatavf(pointTacradDataRef, pointTacrad, 0, 8);

		p += sin(pointTacrad[4] * 0.03f) * pointTacrad[0] * 0.05f;
    q += sin(pointTacrad[5] * 0.03f) * pointTacrad[1] * 0.005f;

    XPLMSetDataf(pDotDataRef, p);
		XPLMSetDataf(qDotDataRef, q);
}

// flightloop-callback that handles everything
static float FlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon)
{
    UpdateDoors();
    UpdateRotor();
    UpdatePilot();
    UpdateSwitches();
    UpdateTransitionalShudder();
    
    return -1.0f;
}

// get doorsLeftPosition
static float GetDoorsLeftPositionCallback(void *inRefcon)
{
    return doorsLeftPosition;
}

// set doorsLeftPosition
static void SetDoorsLeftPositionCallback(void *inRefcon, float newDoorsLeftPosition)
{
    doorsLeftPosition = newDoorsLeftPosition;
}

// get doorsRightPosition
static float GetDoorsRightPositionCallback(void *inRefcon)
{
    return doorsRightPosition;
}

// set doorsRightPosition
static void SetDoorsRightPositionCallback(void *inRefcon, float newDoorsRightPosition)
{
    doorsRightPosition = newDoorsRightPosition;
}

// get adf1
static float GetAdf1Callback(void *inRefcon)
{
    return adf1;
}

// set adf1
static void SetAdf1Callback(void *inRefcon, float newAdf1)
{
    adf1 = newAdf1;
}

// get adf2
static float GetAdf2Callback(void *inRefcon)
{
    return adf2;
}

// set adf2
static void SetAdf2Callback(void *inRefcon, float newAdf2)
{
    adf2 = newAdf2;
}

// get com1
static float GetCom1Callback(void *inRefcon)
{
    return com1;
}

// set com1
static void SetCom1Callback(void *inRefcon, float newCom1)
{
    com1 = newCom1;
}

// get com2
static float GetCom2Callback(void *inRefcon)
{
    return com2;
}

// set com2
static void SetCom2Callback(void *inRefcon, float newCom2)
{
    com2 = newCom2;
}

// get dme
static float GetDmeCallback(void *inRefcon)
{
    return dme;
}

// set dme
static void SetDmeCallback(void *inRefcon, float newDme)
{
    dme = newDme;
}

// get nav1
static float GetNav1Callback(void *inRefcon)
{
    return nav1;
}

// set nav1
static void SetNav1Callback(void *inRefcon, float newNav1)
{
    nav1 = newNav1;
}

// get nav2
static float GetNav2Callback(void *inRefcon)
{
    return nav2;
}

// set nav2
static void SetNav2Callback(void *inRefcon, float newNav2)
{
    nav2 = newNav2;
}

// get tacradsHighMain
static float GetTacradsHighMainCallback(void *inRefcon)
{
    return tacradsHighMain;
}

// set tacradsHighMain
static void SetTacradsHighMainCallback(void *inRefcon, float newTacradsHighMain)
{
    tacradsHighMain = newTacradsHighMain;
}

// get tacradsHighTail
static float GetTacradsHighTailCallback(void *inRefcon)
{
    return tacradsHighTail;
}

// set tacradsHighTail
static void SetTacradsHighTailCallback(void *inRefcon, float newTacradsHighTail)
{
    tacradsHighTail = newTacradsHighTail;
}

// get headHeading
static float GetHeadHeadingCallback(void *inRefcon)
{
    return headHeading;
}

// set headHeading
static void SetHeadHeadingCallback(void *inRefcon, float newHeadHeading)
{
    headHeading = newHeadHeading;
}

// get rotorBladesPitch0
static float GetRotorBladesPitch0Callback(void *inRefcon)
{
    return rotorBladesPitch0;
}

// set rotorBladesPitch0
static void SetRotorBladesPitch0Callback(void *inRefcon, float newRotorBladesPitch0)
{
    rotorBladesPitch0 = newRotorBladesPitch0;
}

// get rotorBladesPitch1
static float GetRotorBladesPitch1Callback(void *inRefcon)
{
    return rotorBladesPitch1;
}

// set rotorBladesPitch1
static void SetRotorBladesPitch1Callback(void *inRefcon, float newRotorBladesPitch1)
{
    rotorBladesPitch1 = newRotorBladesPitch1;
}

// get rotorBladesPitch2
static float GetRotorBladesPitch2Callback(void *inRefcon)
{
    return rotorBladesPitch2;
}

// set rotorBladesPitch2
static void SetRotorBladesPitch2Callback(void *inRefcon, float newRotorBladesPitch2)
{
    rotorBladesPitch2 = newRotorBladesPitch2;
}

// get rotorBladesPitch3
static float GetRotorBladesPitch3Callback(void *inRefcon)
{
    return rotorBladesPitch3;
}

// set rotorBladesPitch3
static void SetRotorBladesPitch3Callback(void *inRefcon, float newRotorBladesPitch3)
{
    rotorBladesPitch3 = newRotorBladesPitch3;
}

// get rotorBladesPitch4
static float GetRotorBladesPitch4Callback(void *inRefcon)
{
    return rotorBladesPitch4;
}

// set rotorBladesPitch4
static void SetRotorBladesPitch4Callback(void *inRefcon, float newRotorBladesPitch4)
{
    rotorBladesPitch4 = newRotorBladesPitch4;
}

// get rotorMutingLowPitch
static float GetRotorMutingLowPitchCallback(void *inRefcon)
{
    return rotorMutingLowPitch;
}

// set rotorMutingLowPitch
static void SetRotorMutingLowPitchCallback(void *inRefcon, float newRotorMutingLowPitch)
{
    rotorMutingLowPitch = newRotorMutingLowPitch;
}

// get rotorMutingLowRoll
static float GetRotorMutingLowRollCallback(void *inRefcon)
{
    return rotorMutingLowRoll;
}

// set rotorMutingLowRoll
static void SetRotorMutingLowRollCallback(void *inRefcon, float newRotorMutingLowRoll)
{
    rotorMutingLowRoll = newRotorMutingLowRoll;
}

// get rotorPositionMain
static float GetRotorPositionMainCallback(void *inRefcon)
{
    return rotorPositionMain;
}

// set rotorPositionMain
static void SetRotorPositionMainCallback(void *inRefcon, float newRotorPositionMain)
{
    rotorPositionMain = newRotorPositionMain;
}

// get rotorPositionMainMuting
static float GetRotorPositionMainMutingCallback(void *inRefcon)
{
    return rotorPositionMainMuting;
}

// set rotorPositionMainMuting
static void SetRotorPositionMainMutingCallback(void *inRefcon, float newRotorPositionMainMuting)
{
    rotorPositionMainMuting = newRotorPositionMainMuting;
}

// get rotorPositionTail
static float GetRotorPositionTailCallback(void *inRefcon)
{
    return rotorPositionTail;
}

// set rotorPositionTail
static void SetRotorPositionTailCallback(void *inRefcon, float newRotorPositionTail)
{
    rotorPositionTail = newRotorPositionTail;
}

// get rotorPositionTailMuting
static float GetRotorPositionTailMutingCallback(void *inRefcon)
{
    return rotorPositionTailMuting;
}

// set rotorPositionTailMuting
static void SetRotorPositionTailMutingCallback(void *inRefcon, float newRotorPositionTailMuting)
{
    rotorPositionTailMuting = newRotorPositionTailMuting;
}

// get rotorPositionMainFpsMuting
static float GetRotorPositionMainFpsMutingCallback(void *inRefcon)
{
    return rotorPositionMainFpsMuting;
}

// set rotorPositionMainFpsMuting
static void SetRotorPositionMainFpsMutingCallback(void *inRefcon, float newRotorPositionMainFpsMuting)
{
    rotorPositionMainFpsMuting = newRotorPositionMainFpsMuting;
}

// get rotorPositionTailFpsMuting
static float GetRotorPositionTailFpsMutingCallback(void *inRefcon)
{
    return rotorPositionTailFpsMuting;
}

// set rotorPositionTailFpsMuting
static void SetRotorPositionTailFpsMutingCallback(void *inRefcon, float newRotorPositionTailFpsMuting)
{
    rotorPositionTailFpsMuting = newRotorPositionTailFpsMuting;
}

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{
    // set plugin info
    strcpy(outName, NAME);
    strcpy(outSig, "de.bwravencl." NAME_LOWERCASE);
    strcpy(outDesc, NAME " provides advanced animations for the Hughes 500D!");

    // register datarefs
    doorsLeftPositionDataRef = XPLMRegisterDataAccessor("abb/doors/left/cockpit/position", xplmType_Float, 1, NULL, NULL, GetDoorsLeftPositionCallback, SetDoorsLeftPositionCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    doorsRightPositionDataRef = XPLMRegisterDataAccessor("abb/doors/right/cockpit/position", xplmType_Float, 1, NULL, NULL, GetDoorsRightPositionCallback, SetDoorsRightPositionCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    adf1DataRef = XPLMRegisterDataAccessor("abb/flags/audio/panel/adf1", xplmType_Float, 1, NULL, NULL, GetAdf1Callback, SetAdf1Callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    adf2DataRef = XPLMRegisterDataAccessor("abb/flags/audio/panel/adf2", xplmType_Float, 1, NULL, NULL, GetAdf2Callback, SetAdf2Callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    com1DataRef = XPLMRegisterDataAccessor("abb/flags/audio/panel/com1", xplmType_Float, 1, NULL, NULL, GetCom1Callback, SetCom1Callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    com2DataRef = XPLMRegisterDataAccessor("abb/flags/audio/panel/com2", xplmType_Float, 1, NULL, NULL, GetCom2Callback, SetCom2Callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    dmeDataRef = XPLMRegisterDataAccessor("abb/flags/audio/panel/dme", xplmType_Float, 1, NULL, NULL, GetDmeCallback, SetDmeCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    nav1DataRef = XPLMRegisterDataAccessor("abb/flags/audio/panel/nav1", xplmType_Float, 1, NULL, NULL, GetNav1Callback, SetNav1Callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    nav2DataRef = XPLMRegisterDataAccessor("abb/flags/audio/panel/nav2", xplmType_Float, 1, NULL, NULL, GetNav2Callback, SetNav2Callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    tacradsHighMainDataRef = XPLMRegisterDataAccessor("abb/flags/rotor/disc/tacrads/high/main", xplmType_Float, 1, NULL, NULL, GetTacradsHighMainCallback, SetTacradsHighMainCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    tacradsHighTailDataRef = XPLMRegisterDataAccessor("abb/flags/rotor/disc/tacrads/high/tail", xplmType_Float, 1, NULL, NULL, GetTacradsHighTailCallback, SetTacradsHighTailCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    headHeadingDataRef = XPLMRegisterDataAccessor("abb/pilot/head/heading/degrees", xplmType_Float, 1, NULL, NULL, GetHeadHeadingCallback, SetHeadHeadingCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    rotorBladesPitch0DataRef = XPLMRegisterDataAccessor("abb/rotor/blades/pitch/0", xplmType_Float, 1, NULL, NULL, GetRotorBladesPitch0Callback, SetRotorBladesPitch0Callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    rotorBladesPitch1DataRef = XPLMRegisterDataAccessor("abb/rotor/blades/pitch/1", xplmType_Float, 1, NULL, NULL, GetRotorBladesPitch1Callback, SetRotorBladesPitch1Callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    rotorBladesPitch2DataRef = XPLMRegisterDataAccessor("abb/rotor/blades/pitch/2", xplmType_Float, 1, NULL, NULL, GetRotorBladesPitch2Callback, SetRotorBladesPitch2Callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    rotorBladesPitch3DataRef = XPLMRegisterDataAccessor("abb/rotor/blades/pitch/3", xplmType_Float, 1, NULL, NULL, GetRotorBladesPitch3Callback, SetRotorBladesPitch3Callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    rotorBladesPitch4DataRef = XPLMRegisterDataAccessor("abb/rotor/blades/pitch/4", xplmType_Float, 1, NULL, NULL, GetRotorBladesPitch4Callback, SetRotorBladesPitch4Callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    rotorMutingLowPitchDataRef = XPLMRegisterDataAccessor("abb/rotor/disc/tilt/pitch/muting/low", xplmType_Float, 1, NULL, NULL, GetRotorMutingLowPitchCallback, SetRotorMutingLowPitchCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    rotorMutingLowRollDataRef = XPLMRegisterDataAccessor("abb/rotor/disc/tilt/roll/muting/low", xplmType_Float, 1, NULL, NULL, GetRotorMutingLowRollCallback, SetRotorMutingLowRollCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    rotorPositionMainDataRef = XPLMRegisterDataAccessor("abb/rotor/position/degrees/main", xplmType_Float, 1, NULL, NULL, GetRotorPositionMainCallback, SetRotorPositionMainCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    rotorPositionMainMutingDataRef = XPLMRegisterDataAccessor("abb/rotor/position/degrees/main/muting", xplmType_Float, 1, NULL, NULL, GetRotorPositionMainMutingCallback, SetRotorPositionMainMutingCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    rotorPositionTailDataRef = XPLMRegisterDataAccessor("abb/rotor/position/degrees/tail", xplmType_Float, 1, NULL, NULL, GetRotorPositionTailCallback, SetRotorPositionTailCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    rotorPositionTailMutingDataRef = XPLMRegisterDataAccessor("abb/rotor/position/degrees/tail/muting", xplmType_Float, 1, NULL, NULL, GetRotorPositionTailMutingCallback, SetRotorPositionTailMutingCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    rotorPositionMainFpsMutingDataRef = XPLMRegisterDataAccessor("abb/rotor/position/main/fps/muting", xplmType_Float, 1, NULL, NULL, GetRotorPositionMainFpsMutingCallback, SetRotorPositionMainFpsMutingCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    rotorPositionTailFpsMutingDataRef = XPLMRegisterDataAccessor("abb/rotor/position/tail/fps/muting", xplmType_Float, 1, NULL, NULL, GetRotorPositionTailFpsMutingCallback, SetRotorPositionTailFpsMutingCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    
    // obtain datarefs
    acfNumBladesDataRef = XPLMFindDataRef("sim/aircraft/prop/acf_num_blades");
    acfCyclicAilnDataRef = XPLMFindDataRef("sim/aircraft/vtolcontrols/acf_cyclic_ailn");
    acfCyclicElevDataRef = XPLMFindDataRef("sim/aircraft/vtolcontrols/acf_cyclic_elev");
    audioPanelOutDataRef = XPLMFindDataRef("sim/cockpit/switches/audio_panel_out");
    flaprqstDataRef = XPLMFindDataRef("sim/flightmodel/controls/flaprqst");
    cyclicElevDiscTiltDataRef = XPLMFindDataRef("sim/flightmodel/cyclic/cyclic_elev_disc_tilt");
    cyclicAilnDiscTiltDataRef = XPLMFindDataRef("sim/flightmodel/cyclic/cyclic_ailn_disc_tilt");
    pointPitchDegDataRef = XPLMFindDataRef("sim/flightmodel/engine/POINT_pitch_deg");
    pointTacradDataRef = XPLMFindDataRef( "sim/flightmodel/engine/POINT_tacrad");
    ongroundAnyDataRef = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    localXDataRef = XPLMFindDataRef("sim/flightmodel/position/local_x");
    localZDataRef = XPLMFindDataRef("sim/flightmodel/position/local_z");
    phiDataRef = XPLMFindDataRef("sim/flightmodel/position/phi");
    psiDataRef = XPLMFindDataRef("sim/flightmodel/position/psi");
    pDotDataRef = XPLMFindDataRef( "sim/flightmodel/position/P_dot");
    qDotDataRef = XPLMFindDataRef( "sim/flightmodel/position/Q_dot");
    viewXDataRef = XPLMFindDataRef("sim/graphics/view/view_x");
    viewZDataRef = XPLMFindDataRef("sim/graphics/view/view_z");
    yolkPitchRatioDataRef = XPLMFindDataRef("sim/joystick/yolk_pitch_ratio");
    yolkRollRatioDataRef = XPLMFindDataRef("sim/joystick/yolk_roll_ratio");
    frameRatePeriodDataRef = XPLMFindDataRef("sim/operation/misc/frame_rate_period");

    // register flight loop callback
    XPLMRegisterFlightLoopCallback(FlightLoopCallback, -1, NULL);

    return 1;
}

PLUGIN_API void	XPluginStop(void)
{
    // unregister datarefs
    XPLMUnregisterDataAccessor(doorsLeftPositionDataRef);
    XPLMUnregisterDataAccessor(doorsRightPositionDataRef);
    XPLMUnregisterDataAccessor(adf1DataRef);
    XPLMUnregisterDataAccessor(adf2DataRef);
    XPLMUnregisterDataAccessor(com1DataRef);
    XPLMUnregisterDataAccessor(com2DataRef);
    XPLMUnregisterDataAccessor(dmeDataRef);
    XPLMUnregisterDataAccessor(nav1DataRef);
    XPLMUnregisterDataAccessor(nav2DataRef);
    XPLMUnregisterDataAccessor(tacradsHighMainDataRef);
    XPLMUnregisterDataAccessor(tacradsHighTailDataRef);
    XPLMUnregisterDataAccessor(headHeadingDataRef);
    XPLMUnregisterDataAccessor(rotorBladesPitch0DataRef);
    XPLMUnregisterDataAccessor(rotorBladesPitch1DataRef);
    XPLMUnregisterDataAccessor(rotorBladesPitch2DataRef);
    XPLMUnregisterDataAccessor(rotorBladesPitch3DataRef);
    XPLMUnregisterDataAccessor(rotorBladesPitch4DataRef);
    XPLMUnregisterDataAccessor(rotorMutingLowPitchDataRef);
    XPLMUnregisterDataAccessor(rotorMutingLowRollDataRef);
    XPLMUnregisterDataAccessor(rotorPositionMainDataRef);
    XPLMUnregisterDataAccessor(rotorPositionMainMutingDataRef);
    XPLMUnregisterDataAccessor(rotorPositionTailDataRef);
    XPLMUnregisterDataAccessor(rotorPositionTailMutingDataRef);
    XPLMUnregisterDataAccessor(rotorPositionMainFpsMutingDataRef);
    XPLMUnregisterDataAccessor(rotorPositionTailFpsMutingDataRef);
}

PLUGIN_API void XPluginDisable(void)
{
}

PLUGIN_API int XPluginEnable(void)
{
    return 1;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho, long inMessage, void *inParam)
{
}
