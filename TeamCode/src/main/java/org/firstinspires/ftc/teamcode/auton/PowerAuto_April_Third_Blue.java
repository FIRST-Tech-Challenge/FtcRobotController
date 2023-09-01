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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.NewHardwareMap;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


//Start of Program
@Autonomous(name="PowerAuto_Blue_Third", group ="Concept", preselectTeleOp="Power_TeleOp")

public class PowerAuto_April_Third_Blue extends LinearOpMode {

    /* Declare OpMode members. */
    NewHardwareMap robot =   new NewHardwareMap();
    BNO055IMU               imu;                            // IMU device
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;

    private ElapsedTime     runtime = new ElapsedTime();
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
    private boolean sight = false;
    private int drive = 20;
    public double sensorNum = 40;
    public double sensorNumIn = 0;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 4;
    int MIDDLE = 2;
    int RIGHT = 5;

    AprilTagDetection tagOfInterest = null;


    @Override public void runOpMode() {   //driver presses Init button

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {

                }
            });

            telemetry.setMsTransmissionInterval(50);
        robot.init(hardwareMap);

        //robot.CameraPan.setPosition(robot.CameraOne);

        //robot.FreightArm.setPosition(0.9);

        //IMU initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

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

            /*******************************************************************
             *                                                                  *
             *                                                                  *
             *                 Image and Signal Determination                   *
             *                                                                  *
             *                                                                  *
             *******************************************************************/

        //New WaitForStart Function

            while (!isStarted() && !isStopRequested())
            {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if(currentDetections.size() != 0)
                {
                    boolean tagFound = false;

                    for(AprilTagDetection tag : currentDetections)
                    {
                        if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                        {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if(tagFound)
                    {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    }
                    else
                    {
                        telemetry.addLine("Don't see tag of interest :(");

                        if(tagOfInterest == null)
                        {
                            telemetry.addLine("(The tag has never been seen)");
                        }
                        else
                        {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }

                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }

                }

                telemetry.update();
                sleep(20);
            }





            if(tagOfInterest != null)
            {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            }
            else
            {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

        robot.LiftMotor.setPower(1.0);
        sleep(200);
        robot.LiftMotor.setPower(0);

        RobotLog.d("LOGGING START");

        /******************************************************************
         *                                                                 *
         *                                                                 *
         *                     Path planning                               *
         *                                                                 *
         *                                                                 *
         ******************************************************************/
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        //Arm positions are 0.9 for closed and 0.5 for open
        //Drive to junction
        gyroDrive(0.4, 3, 0, 15.0);
        gyroStrafe(STRAFE_SPEED, -30, 0, 15.0);
        gyroDriveLU(0.5, 47, 0, 15.0);
        sleep(200);


        while (!robot.touch3.isPressed()) {
            robot.LiftMotor.setPower(1.0);
        }
        robot.LiftMotor.setPower(0.05);

        //Detect the Junction
        gyroSensorStrafe(0.2, 25, 0, 15.0);
        gyroStrafe(0.4, -2, 0, 15.0);

        sensorNum = robot.sensorRange.getDistance(DistanceUnit.INCH);
        sensorNumIn = 6.5 - sensorNum;
        //gyroStrafe(0.2, -2, 0, 15.0);
        gyroDrive(0.4, sensorNumIn, 0, 15.0);

        sleep(200);
        liftDrive(-0.5, 400, true);

        //Deliver Cone
        robot.Gray.setPower(-1);
        robot.Green.setPower(1);
        sleep(1000);
        robot.Gray.setPower(0);
        robot.Green.setPower(0);
        //sleep(500);

        //Drive to Cone Stack
        gyroDrive(DRIVE_SPEED, -2, 0, 15.0);
        gyroTurn(TURN_SPEED, -90);

            /*while (!robot.touch.isPressed()) {
                robot.LiftMotor.setPower(-0.8);
            }
            robot.LiftMotor.setPower(0);*/

        gyroDriveLD(DRIVE_SPEED, 28, -90, 15.0);
        while (!robot.touch.isPressed()) {
            robot.LiftMotor.setPower(-0.8);
        }
        robot.LiftMotor.setPower(0);

        //Pick up cone stack
        liftDrive(1.0, 230, false);
        //gyroDrive(0.5, 8, 90, 15.0);
        robot.Gray.setPower(1);
        robot.Green.setPower(-1);
        gyroDrive(0.4, 7, -90, 15.0);
        sleep(500);
        robot.Gray.setPower(0);
        robot.Green.setPower(0);
        //sleep(500);
        liftDrive(0.6, 800, true);
        gyroDrive(0.5, -11, -90, 15.0);

        //Deliver the second cone
        gyroTurn(TURN_SPEED, -180);
        sleep(100);
        //robot.LiftMotor.setPower(-0.5);
        //sleep(10);

        //Deliver Cone
        robot.Gray.setPower(-1);
        robot.Green.setPower(1);
        sleep(500);
        robot.Gray.setPower(0);
        robot.Green.setPower(0);
        //sleep(500);

        //Third Cone Code Starts Here

        //gyroDrive(DRIVE_SPEED,-3,180,15.0);
        gyroTurn(TURN_SPEED, -90);
        /*while (!robot.touch.isPressed()) {
            robot.LiftMotor.setPower(-0.8);
        }*/
        robot.LiftMotor.setPower(0);
        sleep(200);
        //Pick up cone stack
        liftDrive(-0.6, 700, false);
        gyroDrive(DRIVE_SPEED, 8, -90, 15.0);
        robot.Gray.setPower(1);
        robot.Green.setPower(-1);
        gyroDrive(0.4, 5, -90, 15.0);
        sleep(500);
        robot.Gray.setPower(0);
        robot.Green.setPower(0);
        //sleep(500);
        liftDrive(0.6, 950, true);
        gyroDrive(0.5, -11, -90, 15.0);

        //Deliver the third cone
        gyroTurn(TURN_SPEED, -180);
        //gyroDrive(DRIVE_SPEED,2,180,15.0);
        sleep(100);
        //robot.LiftMotor.setPower(-0.5);
        //sleep(10);

        //Deliver Cone
        robot.Gray.setPower(-1);
        robot.Green.setPower(1);
        sleep(500);
        robot.Gray.setPower(0);
        robot.Green.setPower(0);
        //sleep(500);*/

        gyroDrive(DRIVE_SPEED,-2,-180,15.0);
        gyroTurn(DRIVE_SPEED, -90);
        /*while (!robot.touch.isPressed()) {
            robot.LiftMotor.setPower(-0.8);
        }
        robot.LiftMotor.setPower(0);*/

        //Drive to Target Zone

        if(tagOfInterest.id == RIGHT){
            gyroDriveLD(DRIVE_SPEED, 9, -90, 15.0);
        }
        else if(tagOfInterest.id == MIDDLE){
            gyroDriveLD(DRIVE_SPEED, -14, -90, 15.0);
        }
        else if(tagOfInterest.id == LEFT){
            gyroDriveLD(1.0, -34, -90, 15.0);
        }


    }


    /*******************************************************************
     *                                                                 *
     *                                                                 *
     *                      Lightsaber Library                         *
     *                                                                 *
     *                                                                 *
     *******************************************************************/
        void tagToTelemetry(AprilTagDetection detection)
        {
            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        }

    public void liftDrive(double power, int time, boolean hold) {
        robot.LiftMotor.setPower(power);
        sleep(time);
        if(hold == true) {
            robot.LiftMotor.setPower(0.05);
        }
        else{
            robot.LiftMotor.setPower(0);
        }
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

        robot.LiftMotor.setTargetPosition(LMotorTarget);

        // Turn On RUN_TO_POSITION
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        //runtime.reset();
        robot.LiftMotor.setPower(Math.abs(speed));


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
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
    public void gyroDriveLD ( double speed,
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
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

                if (!robot.touch.isPressed()) {
                    robot.LiftMotor.setPower(-0.8);
                }
                else if (robot.touch.isPressed()) {
                    robot.LiftMotor.setPower(0);
                };

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

    public void gyroDriveLU ( double speed,
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

                if (!robot.touch3.isPressed()) {
                    robot.LiftMotor.setPower(0.6);
                }
                else if (robot.touch3.isPressed()) {
                    robot.LiftMotor.setPower(0);
                };

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
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      FLeftTarget,  FRightTarget, BLeftTarget,  BRightTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.FmotorLeft.getCurrentPosition(),
                        robot.FmotorRight.getCurrentPosition(), robot.BmotorLeft.getCurrentPosition(),
                        robot.BmotorRight.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  frontSpeed, backSpeed);
                telemetry.update();
                RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                        robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                        robot.BmotorRight.getCurrentPosition());
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
            while (opModeIsActive() && (sensorNum > 20 ) &&
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

                //telemetry.addData("deviceName",robot.sensorRange.getDeviceName() );
                //telemetry.addData("range", String.format("%.01f cm", robot.sensorRange.getDistance(DistanceUnit.CM)));
                //telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));

                sensorNum = robot.sensorRange.getDistance(DistanceUnit.CM);
                sensorNumIn = (sensorNum/2.54);

                RobotLog.d("%.01f cm,%.01f in,",robot.sensorRange.getDistance(DistanceUnit.CM),robot.sensorRange.getDistance(DistanceUnit.INCH));

                //telemetry.update();
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