// **** PORTIONS OF THIS LIBRARY WERE DERIVED FROM FIRST EXAMPLES **** 

/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.vision.VisionPortal.CameraState;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import java.util.List;
import java.util.concurrent.TimeUnit;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/**
 * This Vision library is a class with a set of methods to initialize and gather data from cameras 
 * that can provide information to other parts of the robot. 
 *
 * The code branches off of FIRST's ConceptAutoToAprilTagOmni.java.
 *
 **/

public class Vision extends BlocksOpModeCompanion
{
    static private boolean ableToRunVision = false;

    // *** Vision processors ***
    static private VisionPortal visionPortal = null;        // The variable to store our instance of the vision portal. Manages the video source.
    static private boolean manualExposureSet = false;       // Reduces motion blur
    static private AprilTagProcessor aprilTag = null;       // The variable to store our instance of the AprilTag processor. Manages the AprilTag detection process.
    static private TfodProcessor.Builder tfodBuilder = null;                // The variable to store our instance of the TensorFlow Object Detection processor.
    static private TfodProcessor tfod = null;                // The variable to store our instance of the TensorFlow Object Detection processor.

    // *** Tensorflow related variables such as object label to detect in CENTERSTAGE ***
    static private String idTeamProp = "Bolt";
    static private int locationTeamProp  = 0;    // Values 0 = not found, 1 = left, 2 = center, 3 = right

    // *** April Tag variables ***
    static private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    static private int desiredTagID = 0;
    static private double desiredTag_Range = 0;
    static private double desiredTag_Bearing = 0;
    static private double desiredTag_Yaw = 0;
    static private double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    static private boolean targetFound     = false;    // Set to true when an AprilTag target is detected

    // *** Camera variables ***
    static private String frontcamera_name, backcamera_name;  // Holds the names of the cameras as defined in the Hub's configuration file.    
    static private String currentCamera;                         // Holds the value of the current camera streaming.
    static private WebcamName frontcamera_HW, backcamera_HW;      // Holds the hardware handles of the cameras.       
    static private CameraName switchableCamera = null;           // Holds both cameras for the vision portal.
    

    static private int allianceID = 0;  // Used to store the Alliance we are on. 0=if not set, 1=Red, 2=Blue 
    static private int REDALLIANCE =1;
    static private int BLUEALLIANCE = 2;
                                        // Will determine in CENTERSTAGE season with a REV Color sensor on the bot reading a blue or red LEGO brick.
    static private NormalizedColorSensor sensorColorAllianceHW = null;
    static private String sensorColorAllianceNAME;
    static private boolean ableToGetAlliance = false;  // Used to determine if Alliance detection color sensor is able to be used.
    

    @ExportToBlocks (
        heading = "Vision: Initialize",
        color = 255,
        comment = "Initialize vision libraries",
        tooltip = "Wait to start until you see the START displayed.",
        parameterLabels = {"Front Camera Name",
                           "Back Camera Name"
        }
    )
    /**
     * Initialize the AprilTag and Tensorflow processors within the Vision Portal.
     **/
    public static boolean initVision(String _frontcameraname, String _backcameraname) {
        
        frontcamera_name = new String (_frontcameraname);
        backcamera_name = new String (_backcameraname);
         
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        if (aprilTag == null) {
            ableToRunVision = false;
            return false;
        }
        
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);
        
        
        // Create the Tensorflow processor by using a builder.
        tfodBuilder = new TfodProcessor.Builder();
        
        if (tfodBuilder == null) {
            ableToRunVision = false;            
            return false;
        }
        
        // Set the name of the file where the model can be found.
        tfodBuilder.setModelFileName("BP_253_ssd_v2_fpnlite_320x320_metadata.tflite");
        // Set the full ordered list of labels the model is trained to recognize.
        tfodBuilder.setModelLabels(JavaUtil.createListWith(idTeamProp));
        // Set the aspect ratio for the images used when the model was created.
        tfodBuilder.setModelAspectRatio(16 / 9);
        // Create a TfodProcessor by calling build.
        tfod = tfodBuilder.build();
        
        // Next, create a VisionPortal.Builder and set attributes related to the camera.

        frontcamera_HW = hardwareMap.get(WebcamName.class, frontcamera_name);
        backcamera_HW = hardwareMap.get(WebcamName.class, backcamera_name);
        currentCamera = frontcamera_name;
        
        switchableCamera = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(frontcamera_HW, backcamera_HW);
        
        // Create the vision portal by using a builder.
        if (switchableCamera != null) {
            visionPortal = new VisionPortal.Builder().setCamera(switchableCamera).addProcessors(tfod, aprilTag).build();
            ableToRunVision = true;
            
            // Wait for driver to press start
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");        
            
        } else {
            
            // Wait for driver to press start
            telemetry.addData("VISION", "Unable to initialize cameras for vision.");  
            ableToRunVision = false;
            return false;
        
        }
        
        return ableToRunVision;

    }


    @ExportToBlocks (
        heading = "Vision: Initialize with Alliance Detection",
        color = 255,
        comment = "Initialize alliance detection sensor and vision libraries",
        tooltip = "Wait to start until you see the START displayed.",
        parameterLabels = {"Front Camera Name",
                           "Back Camera Name",
                           "Alliance color sensor name"
        }
    )
    /**
     * Initialize the AprilTag and Tensorflow processors within the Vision Portal.
     **/
    public static boolean initVision(String _frontcameraname, String _backcameraname, String _alliancesensorname) {
        float gain = 2;
        
        if (_alliancesensorname.length() > 0) {
            sensorColorAllianceNAME = new String (_alliancesensorname);
            
            // Get a reference to our color sensor object via the FTC hardwaremap of the robot's configuration file. 
            sensorColorAllianceHW = hardwareMap.get(NormalizedColorSensor.class, sensorColorAllianceNAME);
        
            if (sensorColorAllianceHW != null) {
                // Set our color sensor gain.  This is dependent on where the sensor is, where the color item is located and the colors
                sensorColorAllianceHW.setGain(gain);
            } else {
                ableToGetAlliance = false;    
            }

        } else {
            ableToGetAlliance = false;
        }
        
        ableToRunVision = initVision(_frontcameraname, _backcameraname);

        return (ableToRunVision && ableToGetAlliance);
         
    }  // end of initVision() with Alliance detection sensor
    

    @ExportToBlocks (
        heading = "Vision: Set Alliance",
        color = 255,
        comment = "Checks via color sensor what alliance robot is on. Returns [0=Unable to detect, 1=Red Alliance, 2=Blue Alliance]",
        tooltip = "Can call GetAlliance() after this is called."
    )
    /**
     * Indicates the Alliance detected. [0=Unable to detect, 1=Red Alliance, 2=Blue Alliance]
     **/
    public static int setAlliance() {
        
        // Will contain the colors we found from the sensor
        NormalizedRGBA colorsFound = null;
        
        // Array of HSV values. [Array elements: 0-hue, 1-saturation, 2-value. 
        // Reference: http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        final float[] hsvValues = new float[3];    
        float hue = 0;
        
        // If we weren't able to initialize the alliance sensor. Then set to none/0 and return.
        if (!ableToGetAlliance)  {
            allianceID = 0;
            return allianceID;
        }
        
        // Let's turn our sensor's light on if able to on the sensor installed
        if (sensorColorAllianceHW instanceof SwitchableLight) {
            ((SwitchableLight)sensorColorAllianceHW).enableLight(true);            
        }
        
        // Get the normalized colors from the sensor
        colorsFound  = sensorColorAllianceHW.getNormalizedColors();

        // Update the hsvValues array 
        Color.colorToHSV(colorsFound.toColor(), hsvValues);

        // Let's display on the driver station. 
        telemetry.addLine()
             .addData("Red", "%.3f", colorsFound.red)
             .addData("Green", "%.3f", colorsFound.green)
             .addData("Blue", "%.3f", colorsFound.blue);
        telemetry.addLine()
             .addData("Hue", "%.3f", hsvValues[0])
             .addData("Saturation", "%.3f", hsvValues[1])
             .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colorsFound.alpha);
        
        // Let's determine alliance solely on the HUE value
        hue = hsvValues[0];
        
        // See web.archive reference above for why we are using these numbers.
        if (hue >= 200 && hue <= 300)
            allianceID = BLUEALLIANCE;
        else
            allianceID = REDALLIANCE;
                
        // If we were able to turn the light on, then let's turn it off.
         if (sensorColorAllianceHW instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight)sensorColorAllianceHW;
            if (light != null) {
                if (light.isLightOn()) {
                    light.enableLight(false);
                }
            }
         }

        return allianceID;
         
    }  // end of setAlliance() 
    
    @ExportToBlocks (
        heading = "Vision: Get Alliance",
        color = 255,
        comment = "Returns [0=Unable to detect, 1=Red Alliance, 2=Blue Alliance]",
        tooltip = "Need to call InitVision() with sensor and SetAlliance() BEFORE this is called."
    )
    /**
     * Indicates the Alliance detected. [0=Unable to detect, 1=Red Alliance, 2=Blue Alliance]
     **/
    public static int getAlliance() {
        
        return allianceID;
        
    }  // end of getAlliance()
    
    
    @ExportToBlocks (
        heading = "Vision: Get Team Prop Location",
        color = 255,
        comment = "Indicates whether the Team Prop is in sight.",
        tooltip = "Returns an integer [1=Left,2=Center, 3=Right]",
        parameterLabels = {"Tensorflow Label",
                           "Pixel location to split Center and Right locations"
        }
    )
    public static int getTeamPropLocation(String DESIRED_TFOD_LABEL, int splitCenterRight) 
    {
        locationTeamProp = 1;  // Default is Left
        
        if (tfod == null) {
            return locationTeamProp;
        }
        
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        if (currentRecognitions == null) {
            return locationTeamProp;
        }
        
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (DESIRED_TFOD_LABEL.equalsIgnoreCase(recognition.getLabel())) {
                
                telemetry.addData("Requested Object Detected", recognition.getLabel());
                
                if (x <= splitCenterRight ) {
                    locationTeamProp = 2;
                    telemetry.addData("Object Location", "CENTER");
                    
                } else {
                    locationTeamProp = 3;
                    telemetry.addData("Object Location", "RIGHT");                    
                }
            }

        }   // end for() loop

        return locationTeamProp;
    }


    @ExportToBlocks (
        heading = "Vision: Is Tag Visible",
        color = 255,
        comment = "Indicates whether the specified April Tag is in sight.",
        tooltip = "Returns a TRUE or FALSE",
        parameterLabels = {"Desired Tag ID"
        }
    )
    public static boolean isTagVisible(int DESIRED_TAG_ID) 
    {
        targetFound = false;
        desiredTag  = null;

        if (aprilTag == null) {
            targetFound = false;
            return targetFound;
        }
        
        // Use low exposure time to reduce motion blur
        if (!manualExposureSet) {
            manualExposureSet = setManualExposure(6, 250); 
        }
        
        // Get the latest list of detected tags
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        // Make sure we found any detections
        if (currentDetections == null) {
            targetFound = false;
            return targetFound;
        }
        
        // Step through the list of detected tags and look for a matching tag        
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;                    
                    break; // don't look any further.
                    
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        }
        
        return targetFound;
    }

    @ExportToBlocks (
        heading = "Vision: Get Tag Info",
        color = 255,
        comment = "Provides an AprilTagDetection structure for the detected April Tag.",
        tooltip = "Check for NULL.",
        parameterLabels = {"Desired Tag ID"
        }
    )
    public static boolean getTagInfo(int DESIRED_TAG_ID) {
        targetFound = false;
        desiredTag  = null;

        if (aprilTag == null) {
            targetFound = false;
            return targetFound;
        }

        // Use low exposure time to reduce motion blur
        if (!manualExposureSet) {
            manualExposureSet = setManualExposure(6, 250); 
        }
        
       // Get the latest list of detected tags
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
              
        // Make sure we found any detections
        if (currentDetections == null) {
            targetFound = false;
            return targetFound;
        }

        // Step through the list of detected tags and look for a matching tag        
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break; // don't look any further.
                    
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            
            desiredTag_Range = desiredTag.ftcPose.range;
            desiredTag_Bearing = desiredTag.ftcPose.bearing;
            desiredTag_Yaw = desiredTag.ftcPose.yaw;            
        } else {
            desiredTag_Range = desiredTag.ftcPose.range;
            desiredTag_Bearing = desiredTag.ftcPose.bearing;
            desiredTag_Yaw = desiredTag.ftcPose.yaw;            
        }
        
        return targetFound;
    }  // end of getLatestTagInfo()

    @ExportToBlocks (
        heading = "Vision: Get Tag Range",
        color = 255,
        comment = "Returns the Range only",
        tooltip = "Must call the getLatestTagInfo",
        parameterLabels = {"Desired Tag ID"
        }
    )
    /**
     * Returns the range of the desired tag
     */
    public static double getTagRange(int DESIRED_TAG_ID) {
        
        // Make sure we a tag pointer
        if (desiredTag == null) {
            return 0.0;
        }

        if (DESIRED_TAG_ID == desiredTag.id) {
            return desiredTag.ftcPose.range;
        } else {
            return 0;
        }
            
    }  // end of getDesiredTagRange()
    
    @ExportToBlocks (
        heading = "Vision: Get Tag Bearing",
        color = 255,
        comment = "Returns the Bearing only",
        tooltip = "Must call the getLatestTagInfo",
        parameterLabels = {"Desired Tag ID"
        }
    )
    /**
     * Returns the bearing of the desired tag
     */
    public static double getTagBearing(int DESIRED_TAG_ID) {

        // Make sure we a tag pointer
        if (desiredTag == null) {
            return 0.0;
        }

        if (DESIRED_TAG_ID == desiredTag.id) {
            return desiredTag.ftcPose.bearing;
        } else {
            return 0;
        }
    }  // end of getDesiredTagBearing()
    
    @ExportToBlocks (
        heading = "Vision: Get Tag Yaw",
        color = 255,
        comment = "Returns the Yaw only",
        tooltip = "Must call the getLatestTagInfo",
        parameterLabels = {"Desired Tag ID"
        }
    )
    /**
     * Returns the yaw of the desired tag
     */
    public static double getTagYaw(int DESIRED_TAG_ID) {
        
        // Make sure we a tag pointer
        if (desiredTag == null) {
            return 0.0;
        }

        if (DESIRED_TAG_ID == desiredTag.id) {
            return desiredTag.ftcPose.yaw;
        } else {
            return 0;
        }
    }  // end of getDesiredTagYaw()

    @ExportToBlocks (
        heading = "CENTERSTAGE Vision: Get Desired Tag",
        color = 255,
        comment = "Returns the specific AprilTag ID when provided the AllianceID  and Target [1.",
        tooltip = "Must have setAlliance.  Returns 0 if not able to determine.",
        parameterLabels = {    "Alliance ID [1=Red, 2=Blue]",
                            "Target Goal [1=Backdrop Left, 2= Backdrop Center, 3=Backdrop Right, 4=Audience Wall near HP]"
        }
    )
    /**
     * Returns the desired tag ID
     */
    public static int getCENTERSTAGEDesiredTag(int _allianceID, int _targetGoal) {

        if (_allianceID == BLUEALLIANCE && _targetGoal == 1){
            desiredTagID = 1;
        } else if ((_allianceID == REDALLIANCE && _targetGoal == 1)){
            desiredTagID = 1 + 3;
        } else if (_allianceID == BLUEALLIANCE && _targetGoal == 2){
            desiredTagID = 2;
        } else if (_allianceID == REDALLIANCE && _targetGoal == 2){
            desiredTagID = 2 + 3;
        } else if (_allianceID == BLUEALLIANCE && _targetGoal == 3){
            desiredTagID = 3;
        } else if (_allianceID == REDALLIANCE && _targetGoal == 3){
            desiredTagID = 3 + 3;
        } else if (_allianceID == BLUEALLIANCE && _targetGoal == 4){
            desiredTagID = 7;
        } else if (_allianceID == REDALLIANCE && _targetGoal == 4) {
            desiredTagID = 7 + 2;
        } else {
            desiredTagID = 0;
        }
        
        return desiredTagID;

    }  // end of getCENTERSTAGEDesiredTag()
    
    
        /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
     
     EXAMPLE: setManualExposure(6, 250);  // Use low exposure time to reduce motion blur.  NEEDS TO BE CALLED WHEN STREAMING THUS IN THE MAIN LOOP)
 
    */
    public static boolean setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            manualExposureSet = false;
            return manualExposureSet;
        }

        // Set camera controls 
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        if (exposureControl == null) {
            manualExposureSet = false;
            return manualExposureSet;
        }

        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        if (gainControl == null) {
            manualExposureSet = false;
            return manualExposureSet;
        }

        gainControl.setGain(gain);
        
        return manualExposureSet;

    }
    
    @ExportToBlocks (
        heading = "Vision: Switch cameras",
        color = 255,
        comment = "Switches the based on input.",
        tooltip = "Defaults to WebCam 1",
        parameterLabels = {"WebCam Name"
        }
    )
     /**
     * Set the active camera according to input.
     */
    public static void doCameraSwitching(String requestedCamera) {
        
        telemetry.addData("VISION","Request to switch cameras.");   

        if (visionPortal == null) {
            telemetry.addData("VISION","Vision Portal is not initialized.");
            return;
        }
        
        telemetry.addData("VISION",requestedCamera);   
        
        if (visionPortal.getCameraState() == CameraState.STREAMING) {
        
            telemetry.addData("VISION","Streaming...");   
        
            if (frontcamera_name.equalsIgnoreCase(requestedCamera)) {
                telemetry.addData("VISION","Requesting front camera.");  
                
                if (!frontcamera_name.equalsIgnoreCase(currentCamera)) {
                    telemetry.addData("VISION","Switching to front camera.");
                    
                    visionPortal.setActiveCamera(frontcamera_HW);
                    currentCamera = frontcamera_name;
                    
                }
            } else if (backcamera_name.equalsIgnoreCase(requestedCamera) ) {
                telemetry.addData("VISION","Requesting back camera.");                  
                
                if (! backcamera_name.equalsIgnoreCase(currentCamera)) {
                    telemetry.addData("VISION","Switching to back camera.");                    
                    
                    visionPortal.setActiveCamera(backcamera_HW);
                    currentCamera = backcamera_name;
                }
            }

        }

    }   // end method doCameraSwitching()
    
    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        
        if (aprilTag == null) {
            return;
        }
    
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        if (currentDetections == null) {
            return;
        }
        
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        
        if (tfod == null) {
            return;
        }

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        if (currentRecognitions == null) {
            return;
        }

        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()


}
