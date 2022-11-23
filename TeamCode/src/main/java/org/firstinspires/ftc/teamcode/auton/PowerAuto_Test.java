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

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.NewHardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.Date;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.locks.Lock;
import com.qualcomm.hardware.bosch.BNO055IMU;

import java.lang.Math.*;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;



/**
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
@Autonomous(name="PowerAuto_Test", group ="Concept", preselectTeleOp="Power_TeleOp")

public class PowerAuto_Test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Bolt", //yoda
            "2 Bulb", //stormtrooper
            "3 Panel" //tie fighter
    };

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
            "Aa/n2JX/////AAABmS4qOc3rA0BbhY8eFeyaC9YC7jI+cXpqw/nsNuQ6xqHa6GXor3NmLRaRozXHNXMCSwqIH4pQNYBPYvsur4XEuCmq8uEciP6ybVn2U2VZpe16pOkECunnIut6zpoKwz286I+I3aZxgxUsCF0ER3S2bbugrV0HyA8d9jzkaYe9lSuTWTovzhvQhM7Qdso9uI5iZphRyH8qqg6zdYf9WfzQZJ4I2WTVDTd9ZPrB9jM6M2q8rUm9Sh0P//fx0N/tphz9EWXW0p8/UtRC82crWOnW7b7ZMl3Ud+K+AJxVC/PxOcbkbAAF2Fbr6y9mQ7ptrbfy+4U26isGqXbR9JzAQLp/MrJDP0k0zNqZA4u/275QkyLi";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    /* Declare OpMode members. */
    NewHardwareMap robot =   new NewHardwareMap();
    BNO055IMU               imu;                            // IMU device
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;

    static final double     PI = Math.PI;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP 0.025
    static final double     WHEEL_DIAMETER_INCHES   = 3.78;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     STRAFE_SPEED            = 0.6;
    static final double     TURN_SPEED              = 0.25;
    static final double     ROBOT_WIDTH = 13.7;
    static final double     TURN_VALUE = 90;
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.05;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.025;     // Larger is more responsive, but also less stable

    private double skystoneX = 0.0;
    private double skystoneY = 0.0;
    private double skystoneZ = 0.0;

    private VectorF trans = null;
    private boolean cameraStop = true;
    private boolean stackCameraStop = true;
    private int panNumber = 400000;
    private int targetZone = 1;
    private float cameraNumber = 0;
    private int positionNum = 3;
    private int sleepNum = 0;
    /*private int xSubtract = 8;
    private double deliverSS = 0;
    private double pos = 0;
    private double depotNum = 0;
    private double newDeliver = 0;*/
    private boolean sight = false;
    private int drive = 20;
    public double sensorNum = 40;
    public double sensorNumIn = 0;


    @Override public void runOpMode() {   //driver presses Init button
        VuforiaLocalizer.Parameters vparameters = new VuforiaLocalizer.Parameters();

        vparameters.vuforiaLicenseKey = VUFORIA_KEY;
        vparameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(vparameters);

        initTfod();

        robot.init(hardwareMap);

        //robot.CameraPan.setPosition(robot.CameraOne);

        robot.FreightArm.setPosition(0.9);

        //IMU initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        /*// get a reference to our digitalTouch object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "LiftDownLimit");
        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);*/

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "faster than a fish in the Smoky Mountains");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        waitForStart();

        RobotLog.d("LOGGING START");
        /*******************************************************************
         *                                                                  *
         *                                                                  *
         *                 Image and Signal Determination                   *
         *                                                                  *
         *                                                                  *
         *******************************************************************/

        //TFod detection code
        /*    if (opModeIsActive()) {
            while (opModeIsActive() && (stackCameraStop)) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                            if (recognition.getLabel().equals("1 Bolt")) {
                                  telemetry.addData("Image", "1 Bolt");
                                  targetZone = 1;
                              } else if (recognition.getLabel().equals("2 Bulb")) {
                                  telemetry.addData("Image", "2 Bulb");
                                  targetZone = 2;
                              } else if (recognition.getLabel().equals("3 Panel")){
                                  telemetry.addData("Image", "3 Panel");
                                  targetZone = 3;
                              }

                        }
                        telemetry.update();
                    }
                }


               cameraNumber += 1;
               if (cameraNumber == 50000) {
                    stackCameraStop = false;
                    cameraNumber = 0;
                      }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }*/

        /*while(opModeIsActive()) {
        // generic DistanceSensor methods.
        telemetry.addData("deviceName",robot.sensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f cm", robot.sensorRange.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));

        RobotLog.d("%.01f cm,%.01f in,",robot.sensorRange.getDistance(DistanceUnit.CM),robot.sensorRange.getDistance(DistanceUnit.INCH));

        telemetry.update();
        }*/

        /******************************************************************
         *                                                                 *
         *                                                                 *
         *                     Path planning                               *
         *                                                                 *
         *                                                                 *
         ******************************************************************/
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)


        /*gyroSensorStrafe(0.2, -15, 0, 15.0);
        gyroStrafe(0.4, -4, 0, 15.0);
        gyroDrive(0.4, sensorNumIn-4, 0, 15.0);
        robot.Gray.setPower(1);
        robot.Green.setPower(-1);
        sleep(1000);
        robot.Gray.setPower(0);
        robot.Green.setPower(0);*/
        runtime.reset();
        /*TestThread t = new TestThread(this, runtime);
        t.start();*/

    }


    /*******************************************************************
     *                                                                 *
     *                                                                 *
     *                      Lightsaber Library                         *
     *                                                                 *
     *                                                                 *
     *******************************************************************/
     /*class TestThread extends Thread {
         private LinearOpMode opMode;
         TestThread(LinearOpMode opmode, DistanceSensor sensorRange) {
             opMode = opmode;
         }
        public void run() {
         while(opMode.opModeIsActive()){
             telemetry.addData("Status", "Run Time: " + runtime.toString());
             telemetry.update();
         }
     }
     }*/
    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    public void gyroTelem() {
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    public void driveStraight(double speed,double inches,double timeoutS){                                                                                                                                                                                              //:(
        RobotLog.d("DS START");
        encoderDrive(speed, inches, inches, inches, inches, timeoutS);
        RobotLog.d("DS STOP");
    }

    public void strafe(double speed,double inches,double timeoutS){                                                                                                                                                                                              //Drive like a programmer
        encoderDrive(speed, inches, -inches, -inches, inches, timeoutS);
    }

    public void turnLeft (double speed,double degrees, double timeoutS){
        double inches = (PI/180) * (degrees)*  (ROBOT_WIDTH / 2);
        encoderDrive(speed, -inches, inches, -inches, inches, timeoutS);
    }

    public void turnRight (double speed, double degrees, double timeoutS){
        turnLeft (speed, -degrees, timeoutS);
    }

    public void drivePower(double speed, int timer, double timeoutS){
        robot.FmotorLeft.setPower(-speed*0.5);
        robot.FmotorRight.setPower(-speed*0.5);
        robot.BmotorLeft.setPower(-speed*0.5);
        robot.BmotorRight.setPower(-speed*0.5);
        sleep(timer);
        robot.FmotorLeft.setPower(0);
        robot.FmotorRight.setPower(0);
        robot.BmotorLeft.setPower(0);
        robot.BmotorRight.setPower(0);
    }


    public void fPull (double speed,int timer, double timeoutS){
        robot.FmotorLeft.setPower(-speed*0.25);
        robot.FmotorRight.setPower(-speed);
        robot.BmotorLeft.setPower(-speed*0.25);
        robot.BmotorRight.setPower(-speed);
        sleep(timer);
        robot.FmotorLeft.setPower(0);
        robot.FmotorRight.setPower(0);
        robot.BmotorLeft.setPower(0);
        robot.BmotorRight.setPower(0);
    }

    /*public void motorLift (double power,int timer) {
        Shooter.setPower(power);
        sleep(timer);
        Shooter.setPower(0.0);
    }*/

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double FLeftInches, double FRightInches, double BLeftInches, double BRightInches, double timeoutS) {                                                                                                                                                                                                                 // :)

        int FLeftTarget;
        int FRightTarget;
        int BRightTarget;
        int BLeftTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            FLeftTarget  = (int)(-FLeftInches * COUNTS_PER_INCH);
            FRightTarget = (int)(-FRightInches * COUNTS_PER_INCH);
            BLeftTarget = (int)(-BLeftInches * COUNTS_PER_INCH);
            BRightTarget = (int)(-BRightInches * COUNTS_PER_INCH);
            //FLeftTarget  = robot.FmotorLeft.getCurrentPosition() + (int)(FLeftInches * COUNTS_PER_INCH);
            //FRightTarget = robot.FmotorRight.getCurrentPosition() + (int)(FRightInches * COUNTS_PER_INCH);
            //BLeftTarget = robot.BmotorLeft.getCurrentPosition() + (int)(BLeftInches * COUNTS_PER_INCH);
            //BRightTarget = robot.BmotorRight.getCurrentPosition() + (int)(BRightInches * COUNTS_PER_INCH);

            robot.FmotorLeft.setTargetPosition(FLeftTarget);
            robot.FmotorRight.setTargetPosition(FRightTarget);
            robot.BmotorLeft.setTargetPosition(BLeftTarget);
            robot.BmotorRight.setTargetPosition(BRightTarget);

            // Turn On RUN_TO_POSITION
            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Turn off RUN_TO_POSITION
            //robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.FmotorLeft.setPower(Math.abs(speed));
            robot.FmotorRight.setPower(Math.abs(speed));
            robot.BmotorLeft.setPower(Math.abs(speed));
            robot.BmotorRight.setPower(Math.abs(speed));


            /*if (FLeftInches <0 && FRightInches <0) { //Backwards
                robot.BmotorLeft.setPower(-speed);
                robot.BmotorRight.setPower(-speed);
            }else if (FLeftInches <0 && FRightInches >0) { //Left
                robot.BmotorLeft.setPower(-speed);
                robot.BmotorRight.setPower(speed);
            }else if (FLeftInches >0 && FRightInches <0) { //Right
                robot.BmotorLeft.setPower(speed);
                robot.BmotorRight.setPower(-speed);
            }else{                                        //Forwards
                robot.BmotorLeft.setPower(speed);
                robot.BmotorRight.setPower(speed);
            }*/

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Target: ", "to %7d :%7d :%7d :%7d", FLeftTarget,  FRightTarget, BLeftTarget, BRightTarget);
                telemetry.addData("Postion:", "at %7d :%7d :%7d :%7d", robot.FmotorLeft.getCurrentPosition(),
                        robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                        robot.BmotorRight.getCurrentPosition());
                telemetry.update();
                RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                        robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                        robot.BmotorRight.getCurrentPosition());
            }

            // Stop all motion;
            robot.FmotorLeft.setPower(0);
            robot.FmotorRight.setPower(0);
            robot.BmotorLeft.setPower(0);
            robot.BmotorRight.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void liftDrive(double speed, double LMotorInches, double timeoutS) {                                                                                                                                                                                                                 // :)

        int LMotorTarget;


        robot.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        LMotorTarget  = (int)(-LMotorInches * COUNTS_PER_INCH);

        //FLeftTarget  = robot.FmotorLeft.getCurrentPosition() + (int)(FLeftInches * COUNTS_PER_INCH);
        //FRightTarget = robot.FmotorRight.getCurrentPosition() + (int)(FRightInches * COUNTS_PER_INCH);
        //BLeftTarget = robot.BmotorLeft.getCurrentPosition() + (int)(BLeftInches * COUNTS_PER_INCH);
        //BRightTarget = robot.BmotorRight.getCurrentPosition() + (int)(BRightInches * COUNTS_PER_INCH);

        robot.LiftMotor.setTargetPosition(LMotorTarget);

        // Turn On RUN_TO_POSITION
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        //runtime.reset();
        robot.LiftMotor.setPower(Math.abs(speed));


            /*if (FLeftInches <0 && FRightInches <0) { //Backwards
                robot.BmotorLeft.setPower(-speed);
                robot.BmotorRight.setPower(-speed);
            }else if (FLeftInches <0 && FRightInches >0) { //Left
                robot.BmotorLeft.setPower(-speed);
                robot.BmotorRight.setPower(speed);
            }else if (FLeftInches >0 && FRightInches <0) { //Right
                robot.BmotorLeft.setPower(speed);
                robot.BmotorRight.setPower(-speed);
            }else{                                        //Forwards
                robot.BmotorLeft.setPower(speed);
                robot.BmotorRight.setPower(speed);
            }*/

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
            /*while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Target: ", "to %7d :%7d :%7d :%7d", FLeftTarget,  FRightTarget, BLeftTarget, BRightTarget);
                telemetry.addData("Postion:", "at %7d :%7d :%7d :%7d", robot.FmotorLeft.getCurrentPosition(),
                                                             robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                                                             robot.BmotorRight.getCurrentPosition());
                telemetry.update();
                RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                                                             robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                                                             robot.BmotorRight.getCurrentPosition());
            }*/

        // Stop all motion;
        robot.LiftMotor.setPower(0);


        // Turn off RUN_TO_POSITION
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sleep(250);   // optional pause after each move
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle, double timeoutS) {

        RobotLog.d("GD START");
        int FLeftTarget;
        int FRightTarget;
        int BRightTarget;
        int BLeftTarget;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            FLeftTarget  = (int)(-distance * COUNTS_PER_INCH);
            FRightTarget = (int)(-distance * COUNTS_PER_INCH);
            BLeftTarget = (int)(-distance * COUNTS_PER_INCH);
            BRightTarget = (int)(-distance * COUNTS_PER_INCH);

            // Set Target and Turn On RUN_TO_POSITION
            robot.FmotorLeft.setTargetPosition(FLeftTarget);
            robot.FmotorRight.setTargetPosition(FRightTarget);
            robot.BmotorLeft.setTargetPosition(BLeftTarget);
            robot.BmotorRight.setTargetPosition(BRightTarget);

            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);;

            // start motion.
            runtime.reset();
            //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.FmotorLeft.setPower(Math.abs(speed));
            robot.FmotorRight.setPower(Math.abs(speed));
            robot.BmotorLeft.setPower(Math.abs(speed));
            robot.BmotorRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy() )) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.FmotorLeft.setPower(leftSpeed);
                robot.BmotorLeft.setPower(leftSpeed);
                robot.FmotorRight.setPower(rightSpeed);
                robot.BmotorRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      FLeftTarget,  FRightTarget, BLeftTarget,  BRightTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.FmotorLeft.getCurrentPosition(),robot.FmotorRight.getCurrentPosition(),
                        robot.BmotorLeft.getCurrentPosition(), robot.BmotorRight.getCurrentPosition());

                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
                RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                        robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                        robot.BmotorRight.getCurrentPosition());
            }

            RobotLog.d("GD STOP");
            // Stop all motion;
            robot.FmotorLeft.setPower(0);
            robot.FmotorRight.setPower(0);
            robot.BmotorLeft.setPower(0);
            robot.BmotorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroDriveAcc ( double speed,
                               double distance,
                               double angle, double timeoutS) {

        RobotLog.d("GD START");
        int FLeftTarget;
        int FRightTarget;
        int BRightTarget;
        int BLeftTarget;
        double CurrPos;

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double  tempLeftSpeed;
        double  tempRightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            FLeftTarget  = (int)(-distance * COUNTS_PER_INCH);
            FRightTarget = (int)(-distance * COUNTS_PER_INCH);
            BLeftTarget = (int)(-distance * COUNTS_PER_INCH);
            BRightTarget = (int)(-distance * COUNTS_PER_INCH);

            // Set Target and Turn On RUN_TO_POSITION
            robot.FmotorLeft.setTargetPosition(FLeftTarget);
            robot.FmotorRight.setTargetPosition(FRightTarget);
            robot.BmotorLeft.setTargetPosition(BLeftTarget);
            robot.BmotorRight.setTargetPosition(BRightTarget);

            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);;

            // start motion.
            runtime.reset();
            //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.FmotorLeft.setPower(Math.abs(speed));
            robot.FmotorRight.setPower(Math.abs(speed));
            robot.BmotorLeft.setPower(Math.abs(speed));
            robot.BmotorRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy() )) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                //Acc and Decc is here.
                CurrPos=Math.abs((robot.FmotorLeft.getCurrentPosition() + robot.FmotorRight.getCurrentPosition()))/2/COUNTS_PER_INCH;
                tempLeftSpeed = leftSpeed;
                tempRightSpeed = rightSpeed;
                /*if (CurrPos < 1)
                {
                    leftSpeed  = 0.2;
                    rightSpeed = 0.2;
                }*/
                if (CurrPos < 3)
                {
                    leftSpeed  = tempLeftSpeed * CurrPos / 3;
                    rightSpeed = tempRightSpeed * CurrPos / 3;
                }
                /*else if  (distance - CurrPos < 1)
                {
                    leftSpeed  = 0.2;
                    rightSpeed = 0.2;
                }*/
                else if ((distance - CurrPos) < 3)
                {
                    leftSpeed  = tempLeftSpeed * (distance - CurrPos) / 3;
                    rightSpeed = tempRightSpeed * (distance - CurrPos) / 3;
                }
                else
                {
                    //Default full speed
                }

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.FmotorLeft.setPower(leftSpeed);
                robot.BmotorLeft.setPower(leftSpeed);
                robot.FmotorRight.setPower(rightSpeed);
                robot.BmotorRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      FLeftTarget,  FRightTarget, BLeftTarget,  BRightTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.FmotorLeft.getCurrentPosition(),robot.FmotorRight.getCurrentPosition(),
                        robot.BmotorLeft.getCurrentPosition(), robot.BmotorRight.getCurrentPosition());

                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
                RobotLog.d("%7d,%7d,%7d,%7d,%7d,%5.2f:%5.2f", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                        robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                        robot.BmotorRight.getCurrentPosition(), leftSpeed, rightSpeed);
            }

            RobotLog.d("GD STOP");
            // Stop all motion;
            robot.FmotorLeft.setPower(0);
            robot.FmotorRight.setPower(0);
            robot.BmotorLeft.setPower(0);
            robot.BmotorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // negative distance for left strafe, positive distance for right strafe

    public void gyroStrafe ( double speed,
                             double distance,
                             double angle, double timeoutS) {

        RobotLog.d("GS START");
        int FLeftTarget;
        int FRightTarget;
        int BRightTarget;
        int BLeftTarget;
        double  max;
        double  error;
        double  steer;
        double  frontSpeed;
        double  backSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            FLeftTarget  = (int)(-distance * COUNTS_PER_INCH);
            FRightTarget = (int)(distance * COUNTS_PER_INCH);
            BLeftTarget = (int)(distance * COUNTS_PER_INCH);
            BRightTarget = (int)(-distance * COUNTS_PER_INCH);

            // Set Target and Turn On RUN_TO_POSITION
            robot.FmotorLeft.setTargetPosition(FLeftTarget);
            robot.FmotorRight.setTargetPosition(FRightTarget);
            robot.BmotorLeft.setTargetPosition(BLeftTarget);
            robot.BmotorRight.setTargetPosition(BRightTarget);

            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);;

            // start motion.
            runtime.reset();
            //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.FmotorLeft.setPower(Math.abs(speed));
            robot.FmotorRight.setPower(Math.abs(speed));
            robot.BmotorLeft.setPower(Math.abs(speed));
            robot.BmotorRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy() )) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                frontSpeed = speed - steer;
                backSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(frontSpeed), Math.abs(backSpeed));
                if (max > 1.0)
                {
                    frontSpeed /= max;
                    backSpeed /= max;
                }

                robot.FmotorLeft.setPower(frontSpeed);
                robot.BmotorLeft.setPower(backSpeed);
                robot.FmotorRight.setPower(frontSpeed);
                robot.BmotorRight.setPower(backSpeed);

                // Display drive status for the driver.
                    /*telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                    telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      FLeftTarget,  FRightTarget, BLeftTarget,  BRightTarget);
                    telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.FmotorLeft.getCurrentPosition(),
                            robot.FmotorRight.getCurrentPosition(), robot.BmotorLeft.getCurrentPosition(),
                            robot.BmotorRight.getCurrentPosition());
                    telemetry.addData("Speed",   "%5.2f:%5.2f",  frontSpeed, backSpeed);
                    telemetry.update();
                    RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                                                             robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                                                             robot.BmotorRight.getCurrentPosition());*/

                telemetry.addData("deviceName",robot.sensorRange.getDeviceName() );
                telemetry.addData("range", String.format("%.01f cm", robot.sensorRange.getDistance(DistanceUnit.CM)));
                telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));

                RobotLog.d("%.01f cm,%.01f in,",robot.sensorRange.getDistance(DistanceUnit.CM),robot.sensorRange.getDistance(DistanceUnit.INCH));

                telemetry.update();
            }

            RobotLog.d("GS STOP");
            // Stop all motion;
            robot.FmotorLeft.setPower(0);
            robot.FmotorRight.setPower(0);
            robot.BmotorLeft.setPower(0);
            robot.BmotorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void gyroSensorStrafe ( double speed,
                                   double distance,
                                   double angle, double timeoutS) {

        RobotLog.d("GS START");
        int FLeftTarget;
        int FRightTarget;
        int BRightTarget;
        int BLeftTarget;
        //double sensorNum;
        double  max;
        double  error;
        double  steer;
        double  frontSpeed;
        double  backSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            FLeftTarget  = (int)(-distance * COUNTS_PER_INCH);
            FRightTarget = (int)(distance * COUNTS_PER_INCH);
            BLeftTarget = (int)(distance * COUNTS_PER_INCH);
            BRightTarget = (int)(-distance * COUNTS_PER_INCH);

            // Set Target and Turn On RUN_TO_POSITION
            robot.FmotorLeft.setTargetPosition(FLeftTarget);
            robot.FmotorRight.setTargetPosition(FRightTarget);
            robot.BmotorLeft.setTargetPosition(BLeftTarget);
            robot.BmotorRight.setTargetPosition(BRightTarget);

            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);;

            // start motion.
            runtime.reset();
            //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.FmotorLeft.setPower(Math.abs(speed));
            robot.FmotorRight.setPower(Math.abs(speed));
            robot.BmotorLeft.setPower(Math.abs(speed));
            robot.BmotorRight.setPower(Math.abs(speed));

            //sensorNum = robot.sensorRange.getDistance(DistanceUnit.CM);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (sensorNum > 30 ) &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy() )) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                frontSpeed = speed - steer;
                backSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(frontSpeed), Math.abs(backSpeed));
                if (max > 1.0)
                {
                    frontSpeed /= max;
                    backSpeed /= max;
                }

                robot.FmotorLeft.setPower(frontSpeed);
                robot.BmotorLeft.setPower(backSpeed);
                robot.FmotorRight.setPower(frontSpeed);
                robot.BmotorRight.setPower(backSpeed);

                // Display drive status for the driver.
                    /*telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                    telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      FLeftTarget,  FRightTarget, BLeftTarget,  BRightTarget);
                    telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.FmotorLeft.getCurrentPosition(),
                            robot.FmotorRight.getCurrentPosition(), robot.BmotorLeft.getCurrentPosition(),
                            robot.BmotorRight.getCurrentPosition());
                    telemetry.addData("Speed",   "%5.2f:%5.2f",  frontSpeed, backSpeed);
                    telemetry.update();
                    RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                                                             robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                                                             robot.BmotorRight.getCurrentPosition());*/

                telemetry.addData("deviceName",robot.sensorRange.getDeviceName() );
                telemetry.addData("range", String.format("%.01f cm", robot.sensorRange.getDistance(DistanceUnit.CM)));
                telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));

                sensorNum = robot.sensorRange.getDistance(DistanceUnit.CM);
                sensorNumIn = robot.sensorRange.getDistance(DistanceUnit.INCH);

                RobotLog.d("%.01f cm,%.01f in,",robot.sensorRange.getDistance(DistanceUnit.CM),robot.sensorRange.getDistance(DistanceUnit.INCH));

                telemetry.update();
            }

            RobotLog.d("GS STOP");
            // Stop all motion;
            robot.FmotorLeft.setPower(0);
            robot.FmotorRight.setPower(0);
            robot.BmotorLeft.setPower(0);
            robot.BmotorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle) {
        RobotLog.d("GT START");
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.

            telemetry.update();
        }
        RobotLog.d("GT STOP");
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.FmotorLeft.setPower(0);
        robot.FmotorRight.setPower(0);
        robot.BmotorLeft.setPower(0);
        robot.BmotorRight.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    public boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.FmotorLeft.setPower(-leftSpeed);
        robot.BmotorLeft.setPower(-leftSpeed);
        robot.FmotorRight.setPower(-rightSpeed);
        robot.BmotorRight.setPower(-rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        RobotLog.d("%5.2f,%5.2f,%5.2f,%5.2f, %5.2f", angle, error, steer, leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1.0, 1.0);
    }

}