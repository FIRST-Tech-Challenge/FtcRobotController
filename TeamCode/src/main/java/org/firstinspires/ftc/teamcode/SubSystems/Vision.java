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

package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

import java.util.ArrayList;
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

    public enum VISION_STATE {
        TFOD_INIT,
        TFOD_ACTIVE,
        TFOD_RUNNING,
        NAVIGATION_INIT,
        NAVIGATION_ACTIVE,
        NAVIGATION_RUNNING,
        INACTIVE
    }
    public VISION_STATE visionState = VISION_STATE.INACTIVE;

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

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

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;
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

    public String visibleTargetName = "";

    public Pose2d poseVuforia = new Pose2d (0,0,0);
    public Pose2d vuforiaCameraCorrection = new Pose2d( 0,-3.75, 0);
    //public Pose2d vuforiaLeftCameraCorrection = new Pose2d( -3.75,-3.75, Math.PI);
    //public Pose2d vuforiaRightCameraCorrection = new Pose2d( 3.75,3.75, Math.PI);
    public double vuforiaFirstAngle = 0;
    public double vuforiaSecondAngle = 0;
    public double vuforiaThirdAngle = 0;

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
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    public static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    public String targetLabel = LABELS[2]; // Default "Duck"
    public String detectedLabel = LABELS[2];
    public float detectedLabelLeft, detectedLabelRight, detectedLabelTop, detectedLabelBottom;
    public static float[] targetPosition = {
            //TODO : Update values based on marker location identifier
            10,
            20,
            30
    };

    private TFObjectDetector tfod;
    private List<Recognition> recognitions;
    public GameField.VISION_IDENTIFIED_TARGET targetLevelDetected = GameField.VISION_IDENTIFIED_TARGET.UNKNOWN;

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
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        if (GameField.visionIdentifier == GameField.VISION_IDENTIFIER.MARKER){
            targetLabel = LABELS[3]; //"Marker"
        } else {//if (GameField.visionIdentifier == GameField.VISION_IDENTIFIER.DUCK)
            targetLabel = LABELS[2];
        }
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
            tfod.setZoom(2.0, 16.0/9.0);
            recognitions = tfod.getUpdatedRecognitions();
        }
    }

    /**
     * Run Tensor flow algorithm.
     * This is to be run till the play button is pressed.. the last target zone identified is returned.
     * @return
     */
    public GameField.VISION_IDENTIFIED_TARGET runVuforiaTensorFlow() {
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
                    targetLevelDetected = GameField.VISION_IDENTIFIED_TARGET.LEVEL3;
                } else {
                    // list is not empty.
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    /* step through the list of recognitions and display boundary info.
                    for (Recognition recognition : recognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }*/

                    for (Recognition recognition : recognitions) {
                        // check label to see which target zone to go after.
                        detectedLabel = recognition.getLabel();
                        detectedLabelLeft = recognition.getLeft();
                        detectedLabelRight = recognition.getRight();
                        detectedLabelTop = recognition.getTop();
                        detectedLabelBottom = recognition.getBottom();
                        if (recognition.getLabel().equals(targetLabel)) {
                            if (recognition.getLeft() < targetPosition[0]) {
                                targetLevelDetected = GameField.VISION_IDENTIFIED_TARGET.LEVEL1;
                            } else if (recognition.getLeft() < targetPosition[1]) {
                                targetLevelDetected = GameField.VISION_IDENTIFIED_TARGET.LEVEL2;
                            } else {//if (recognition.getLeft() < targetPosition[2])
                                targetLevelDetected = GameField.VISION_IDENTIFIED_TARGET.LEVEL3;
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


    /**
     * Initialize Vuforia for Navigation
     */
    public void setupVuforiaNavigation() {

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");
        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        /*
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
         * with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */

        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        visionState = VISION_STATE.NAVIGATION_INIT;

    }

    /**
     * Activate Vuforia Navigation
     */
    public void activateVuforiaNavigation() {
        /* Note: To use the remote camera preview:
         * AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
         * Tap the preview window to receive a fresh image.
         * It is not permitted to transition to RUN while the camera preview window is active.
         * Either press STOP to exit the OpMode, or use the "options menu" again, and select "Camera Stream" to close the preview window.
         */

        targets.activate();
        visionState = VISION_STATE.NAVIGATION_ACTIVE;
    }

    /**
     * Run Vuforia Navigation
     */
    public void runVuforiaNavigation() {
        visionState = VISION_STATE.NAVIGATION_RUNNING;
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                //telemetry.addData("Visible Target", trackable.getName());
                visibleTargetName = trackable.getName();
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            //telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
            //        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, RADIANS);
            //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            vuforiaFirstAngle=rotation.firstAngle;
            vuforiaSecondAngle=rotation.secondAngle;
            vuforiaThirdAngle=rotation.thirdAngle;
            poseVuforia = (new Pose2d(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, rotation.thirdAngle)).plus(vuforiaCameraCorrection);
            /*if (GameField.playingAlliance == GameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                poseVuforia = (new Pose2d(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, rotation.thirdAngle)).plus(vuforiaLeftCameraCorrection);
            } else {
                poseVuforia = (new Pose2d(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, rotation.thirdAngle)).plus(vuforiaRightCameraCorrection);
            }*/

        }
        else {
            //telemetry.addData("Visible Target", "none");
            visibleTargetName = "none";
        }

        //telemetry.update();
     }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    /**
     * Stop Vuforia Navigation
     */
    public void deactivateVuforiaNavigation() {
        // Disable Tracking when we are done;
        targets.deactivate();
        visionState = VISION_STATE.INACTIVE;

    }

}
