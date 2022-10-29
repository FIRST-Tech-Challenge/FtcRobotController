/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.AutoOpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.List;

/**
 *
 * Vuforia implementation for Hazmat
 *  - Integrates Tensor flow and Navigation to single code
 *  - Updated to manage 2 cameras
 *
 * This 2021-2022 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * This 2020-2021 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the ULTIMATE GOAL FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * There are a total of five image targets for the ULTIMATE GOAL game.
 * Three of the targets are placed in the center of the Red Alliance, Audience (Front),
 * and Blue Alliance perimeter walls.
 *
 * Two additional targets are placed on the perimeter wall, one in front of each Tower Goal.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ultimategoal/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


public class Vision {
    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    public static final Vector2d ORIGIN = new Vector2d(0,0);
    public static final Pose2d ORIGINPOSE = new Pose2d(0,0,Math.toRadians(0));

    // Declare and assign starting pose of robot
    public static final Pose2d STARTPOS_1 =  new Pose2d(-61,7,Math.toRadians(180));
    public static final Pose2d STARTPOS_2 =  new Pose2d(-61,-40,Math.toRadians(180));
    public static final Pose2d STARTPOS_3 =  new Pose2d(61,7,Math.toRadians(0));
    public static final Pose2d STARTPOS_4 =  new Pose2d(61,-40,Math.toRadians(0));

    //Define and declare Playing Alliance
    public enum PLAYING_ALLIANCE{
        RED_ALLIANCE,
        BLUE_ALLIANCE,
    }
    public static PLAYING_ALLIANCE playingAlliance = PLAYING_ALLIANCE.BLUE_ALLIANCE;
    public static double ALLIANCE_FACTOR = 1;


    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        POS1,
        POS2,
        POS3
    }

    //Set a default start position
    public static START_POSITION startPosition = START_POSITION.POS1;

    //Define and declare Robot parking locations
    public enum PARKING_LOCATION{
        ENDPOS1,
        ENDPOS2,
        ENDPOS3
    }

    //Select the different colors on the signal sleeve, and define them here
    public enum VISION_IDENTIFIED_TARGET {
        RED,
        BLUE,
        GREEN,
    };

    public enum VISION_IDENTIFIER{
        MARKER
    };
    public static VISION_IDENTIFIER visionIdentifier = VISION_IDENTIFIER.MARKER;

    //Static fields to pass Pos from Autonomous to TeleOp
    public static boolean poseSetInAutonomous = false;
    public static Pose2d currentPose = new Pose2d();

    public enum VISION_STATE {
        TFOD_INIT,
        TFOD_ACTIVE,
        TFOD_RUNNING,
        INACTIVE
    }
    public VISION_STATE visionState = VISION_STATE.INACTIVE;

    public enum ACTIVE_WEBCAM{
        WEBCAM1,
        WEBCAM2,
    }


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AZME4Mr/////AAABmY+MAyxxT0IileR7JBqaAPsxN2XNYlaGBtEjYaHOlVVTqPQf7NH9eIrosYKKQHPGEXLtJUsdMwZ9e3EXBfy6arulcLPvdpW9bqAB2F2MJJXo35lLA096l/t/LQTi+etVso0Xc5RYkTVSIP3YABp1TeOaF8lCSpjVhPIVW3l/c/XlrnEMPhJk9IgqMEp4P/ifqAqMMMUAIKPEqIrXIv79TvAfdIJig46gfQGaQl5tFHr3nmvMbh/LhFrh5AWAy3B/93cCkOszmYkdHxZStbNB5lMdkTnf3sCnYbQY4jviorfhYrAkqHWH6vNOB9lUt8dOSeHsDtlk33e/6xQgOCNYFN80anYMp82JNDBFX3oyGliV";


    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targets   = null ;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    public  boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    public VuforiaLocalizer.Parameters parameters;

    public List<VuforiaTrackable> allTrackables;

    //Tensor Flow parameters
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "Hazmat1.tflite";
    public static final String[] LABELS = {
            "RED",
            "BLUE",
            "GREEN"
    };


    public String targetLabel1 = LABELS[0]; //RED
    public String targetLabel2 = LABELS[1]; // BLUE
    public String targetLabel3 = LABELS[2]; // GREEN
    public String detectedLabel = "None";
    public float detectedLabelLeft, detectedLabelRight, detectedLabelTop, detectedLabelBottom;
    public static float[] targetPosition = {
            //TODO : Update values based on marker location identifier
            250,
            600,
            1000
    };

    private TFObjectDetector tfod;
    private List<Recognition> recognitions;
    public VISION_IDENTIFIED_TARGET targetLevelDetected = VISION_IDENTIFIED_TARGET.RED;

    /**
     * Initialize the Vuforia localization engine.
     */
    public Vision(HardwareMap hardwareMap, ACTIVE_WEBCAM activeWebcam) {
        activeWebcam = ACTIVE_WEBCAM.WEBCAM1;

        if (activeWebcam == ACTIVE_WEBCAM.WEBCAM1){
            webcamName = hardwareMap.get(WebcamName.class, "Webcam1");
        } /*else { //TODO: Uncomment if using 2 cameras;
            webcamName = hardwareMap.get(WebcamName.class, "Webcam2");
        }
        */

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        initTfod(hardwareMap);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320; //320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        /*if (GameField.visionIdentifier == GameField.VISION_IDENTIFIER.MARKER){
            targetLabel2 = LABELS[3]; //"Marker"
        } else {//if (GameField.visionIdentifier == GameField.VISION_IDENTIFIER.DUCK)
            targetLabel2 = LABELS[2];
        }*/
        visionState = VISION_STATE.TFOD_INIT;

    }

    /**
     * Activate Vuforia Tensor Flow to determine target zone
     * This is to be done at Init in Autonomous mode
     */
    public void activateVuforiaTensorFlow(){
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            visionState = VISION_STATE.TFOD_ACTIVE;

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(1.75, 16.0/9.0);
            tfod.setZoom(1.0, 16.0/9.0);
            recognitions = tfod.getUpdatedRecognitions();
        }
    }

    /**
     * Run Tensor flow algorithm.
     * This is to be run till the play button is pressed.. the last target zone identified is returned.
     * @return
     */



    public VISION_IDENTIFIED_TARGET runVuforiaTensorFlow() {
        visionState = VISION_STATE.TFOD_RUNNING;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            recognitions = tfod.getUpdatedRecognitions();
            if (recognitions != null) {
                //telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (recognitions.size() == 0 ) {
                    // empty list.  no objects recognized.
                    detectedLabel = "None";

                    if(startPosition != START_POSITION.POS1){
                        if (playingAlliance == PLAYING_ALLIANCE.RED_ALLIANCE) {
                            //Depending on the startPos and alliance, what is the camera going to detect
                        } else { //playingAlliance == PLAYING_ALLIANCE.BLUE_ALLIANCE
                            //Depending on the startPos and alliance, what is the camera going to detect
                        }
                    } else if(startPosition != START_POSITION.POS2) {
                        if (playingAlliance == PLAYING_ALLIANCE.RED_ALLIANCE) {
                            //Depending on the startPos and alliance, what is the camera going to detect
                        } else { //playingAlliance == PLAYING_ALLIANCE.BLUE_ALLIANCE
                            //Depending on the startPos and alliance, what is the camera going to detect
                        }
                    } else { //startPosition != START_POSITION.POS3
                        if (playingAlliance == PLAYING_ALLIANCE.RED_ALLIANCE) {
                            //Depending on the startPos and alliance, what is the camera going to detect
                        } else { //playingAlliance == PLAYING_ALLIANCE.BLUE_ALLIANCE
                            //Depending on the startPos and alliance, what is the camera going to detect
                        }
                    }
                } else {
                    // list is not empty.
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    /*
                     step through the list of recognitions and display boundary info.
                    for (Recognition recognition : recognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }
                     */

                    for (Recognition recognition : recognitions) {
                        // check label to see which target zone to go after.
                        detectedLabel = recognition.getLabel();
                        detectedLabelLeft = recognition.getLeft();
                        detectedLabelRight = recognition.getRight();
                        detectedLabelTop = recognition.getTop();
                        detectedLabelBottom = recognition.getBottom();
                        if (recognition.getLabel().equals(LABELS[0]) || recognition.getLabel().equals(LABELS[1]) ||
                                recognition.getLabel().equals(LABELS[2])) {

                            if (recognition.getLeft() < targetPosition[0]) {
                                if(startPosition != START_POSITION.POS1){
                                    if (playingAlliance == PLAYING_ALLIANCE.RED_ALLIANCE) {
                                        //Depending on the startPos and alliance, what is the camera going to detect
                                    } else { //playingAlliance == PLAYING_ALLIANCE.BLUE_ALLIANCE
                                        //Depending on the startPos and alliance, what is the camera going to detect
                                    }
                                } else if(startPosition != START_POSITION.POS2) {
                                    if (playingAlliance == PLAYING_ALLIANCE.RED_ALLIANCE) {
                                        //Depending on the startPos and alliance, what is the camera going to detect
                                    } else { //playingAlliance == PLAYING_ALLIANCE.BLUE_ALLIANCE
                                        //Depending on the startPos and alliance, what is the camera going to detect
                                    }
                                } else { //startPosition != START_POSITION.POS3
                                    if (playingAlliance == PLAYING_ALLIANCE.RED_ALLIANCE) {
                                        //Depending on the startPos and alliance, what is the camera going to detect
                                    } else { //playingAlliance == PLAYING_ALLIANCE.BLUE_ALLIANCE
                                        //Depending on the startPos and alliance, what is the camera going to detect
                                    }
                                }
                            } else if (recognition.getLeft() < targetPosition[1]) {
                                if(startPosition != START_POSITION.POS1){
                                    if (playingAlliance == PLAYING_ALLIANCE.RED_ALLIANCE) {
                                        //Depending on the startPos and alliance, what is the camera going to detect
                                    } else { //playingAlliance == PLAYING_ALLIANCE.BLUE_ALLIANCE
                                        //Depending on the startPos and alliance, what is the camera going to detect
                                    }
                                } else if(startPosition != START_POSITION.POS2) {
                                    if (playingAlliance == PLAYING_ALLIANCE.RED_ALLIANCE) {
                                        //Depending on the startPos and alliance, what is the camera going to detect
                                    } else { //playingAlliance == PLAYING_ALLIANCE.BLUE_ALLIANCE
                                        //Depending on the startPos and alliance, what is the camera going to detect
                                    }
                                } else { //startPosition != START_POSITION.POS3
                                    if (playingAlliance == PLAYING_ALLIANCE.RED_ALLIANCE) {
                                        //Depending on the startPos and alliance, what is the camera going to detect
                                    } else { //playingAlliance == PLAYING_ALLIANCE.BLUE_ALLIANCE
                                        //Depending on the startPos and alliance, what is the camera going to detect
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        return targetLevelDetected;
    }


    /**
     * Stop Tensor Flow algorithm
     */
    public void deactivateVuforiaTensorFlow(){
        if (tfod != null) {
            tfod.shutdown();
            visionState = VISION_STATE.INACTIVE;
        }
    }
}
