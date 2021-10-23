package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

import java.util.List;

@Autonomous(name="NewAutoCamera", group="2021 Ultimate Goal")
public class NewAutoCamera extends LinearOpMode
{
    // Set Timer
    private ElapsedTime  runtime = new ElapsedTime();


    // @TODO: Intialize a variable to store the ring count. You might update this later in the code.
    // camera recognition and tensor flow
    private VuforiaCurrentGame vuforiaUltimateGoal;
    private TfodCurrentGame tfodUltimateGoal;

    // number of rings camera finds
    Recognition recognition;

    //declare variable
    private int ringCount = 0;

    // declare motors
    private DcMotor right;
    private DcMotor left;
    private DcMotor center;
    private CRServo intake;
    private CRServo launcher;
    private Servo claw;
    private Servo arm;
    private Servo flicker;
    private BNO055IMU imu;

    // encoder clicks are originally 1680
    // multiply that by drive train gear ratio (two thirds)
    private static final int ENCODER_CLICKS = 1680;
    private static final double WHEEL_DIAM = 10.0;
    private static final double WHEEL_CIRC = WHEEL_DIAM * Math.PI;
    private static final double DRIVE_GEAR_RATIO = 2/3;
    private static final double CLICKS_PER_CM = ENCODER_CLICKS / WHEEL_CIRC;

    // Declare IMU
    BNO055IMU.Parameters IMU_Parameters;
    ElapsedTime ElapsedTime2;
    double Left_Power;
    double Right_Power;
    double Original_Power;
    float Yaw_Angle = 0;



    @Override
    public void runOpMode()
    {
        // Telemetry to show OpMode is initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // list rings
        List<Recognition> recognitions;
        double index;

        vuforiaUltimateGoal = new VuforiaCurrentGame();
        tfodUltimateGoal = new TfodCurrentGame();

        // Initialize the hardware variables.
        left  = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        center = hardwareMap.get(DcMotor.class, "center");
        intake = hardwareMap.get(CRServo.class, "intake");
        launcher = hardwareMap.get(CRServo.class, "launcher");
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");
        flicker = hardwareMap.get(Servo.class, "flicker");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Initialize motor direction
        right.setDirection(DcMotor.Direction.FORWARD);
        left.setDirection(DcMotor.Direction.REVERSE);
        center.setDirection(DcMotor.Direction.REVERSE);


        // Reset motor encoders
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize motor encoder modes
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
        arm.setDirection(Servo.Direction.FORWARD);
        flicker.setDirection(Servo.Direction.FORWARD);

        // Initialize IMU
        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(IMU_Parameters);

        telemetry.addData("Status", "IMU initialized, calibration started.");
        telemetry.update();

        // Calibrate IMU
        while (!IMU_Calibrated()) {
            telemetry.addData("If calibration ", "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
            telemetry.update();
            // Wait one second before checking calibration status again.
            sleep(1000);
        }

        telemetry.addData("Status", "Calibration Complete");
        telemetry.update();


        // Initialize Vuforia.
        // We need Vuforia to provide TFOD with camera images.
        vuforiaUltimateGoal.initialize(
                "AfacJkb/////AAABmYDDu+vj6knuuMS/g+9F+bwOQBEMB9uDwPrDbCTt+2vv8caD/rZ47CeNGlqKRzd8sEWyv/N5c051wbC9mOBXx0qSafGm7pPqaNbk5B7fF/mXuvBPcsfbxtM1hHrWtS+xxZKafEyC63AAIBWp6nPsbhV/LwN8F65750TQg9WZA4yuFGpAxuNxSW61KdKs6kU12MzWz0chRLtZLWbFJiwB4jy5KteBeL9+PzTgw4FWBfVnSHxnC2sVCyk/O7ZxPTD+v0bKQLusNNW3CV1U+PXWXjWr3Najo2v660Ui4O999r12jmtwSARQWMmzh9EaaftO2R79Bv4y8RKlhV4G2k/E25szK4PI/B/YzrGtxepL3zNQ", // vuforiaLicenseKey
                VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                0, // xAngle
                -90, // yAngle
                0, // zAngle
                true); // useCompetitionFieldTargetLocations

        // Set min confidence threshold
        tfodUltimateGoal.initialize(vuforiaUltimateGoal, 0.6F, true, true);

        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodUltimateGoal.activate();

        // Enable following block to zoom in on target.
        tfodUltimateGoal.setZoom(1.6, 16.0 / 9.0);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();


        telemetry.addData("status","init" );
        telemetry.update();


        // OpMode needs waitForStart(); and while (opModeIsActive()) {...}
        waitForStart();



        telemetry.addData("status","op mode started" );
        telemetry.update();

        // list number of rings found
        recognitions = tfodUltimateGoal.getRecognitions();

        // If list is empty, inform the user. Otherwise, go
        // through list and display info for each recognition.
        if (recognitions.size() == 0) {

            telemetry.addData("TFOD", "No items detected.");
            telemetry.addData("Target Zone", "A");

        } else {
            index = 0;
            // Iterate through list and call a function to
            // display info for each recognized object.
            for (Recognition recognition_item : recognitions) {
                recognition = recognition_item;

                if (recognition.getLabel().equals("Single")){
                    ringCount=1;
                }else{
                    ringCount=4;
                }
                // Increment index.
                index = index + 1;
            }
        }

        telemetry.addData("status","ringCount,  %7d", ringCount );
        telemetry.update();
        sleep(1000);

        // Deactivate TFOD.
        tfodUltimateGoal.deactivate();
        vuforiaUltimateGoal.close();
        tfodUltimateGoal.close();

        //Automonous Drive Code Starts Here
        //was 78 78
        double SoupTimeLol = -0.63;
        //was -0.6
        launcher.setPower(SoupTimeLol);
        claw.setPosition(0.9);
        driveBot(75,75,0.6,5.0);

        sleep(1000);

        /*center.setTargetPosition(1680);
        center.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        center.setPower(0.45);
        sleep(1000);
        center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        center.setPower(0);

        */
        //turn to line super
        launcher.setPower(SoupTimeLol);
        driveBot(-2.3, 2.3, 0.4, 5.0);
        sleep(1000);

        // Shoot rings
        flicker.setPosition(0);
        sleep(1000); // spinup the flywheel

        // Flick ring 1
        launcher.setPower(SoupTimeLol);
        flicker.setPosition(0.5);
        sleep(500);
        flicker.setPosition(0);
        sleep(3000);


        // Flick ring 2
        launcher.setPower(SoupTimeLol);
        flicker.setPosition(0.5);
        sleep(500);
        flicker.setPosition(0);
        sleep(1000);

        //turn back
        //turn(true, 1);


        if (ringCount == 1) {

            driveBot(15.4,15.4,0.4,5.0);
            telemetry.addData("status","first run to position called" );
            telemetry.addData("status", left.getMode() );
            telemetry.addData("status","left motor,  %7d", left.getCurrentPosition() );
            telemetry.addData("status","right motor,  %7d", right.getCurrentPosition() );
            telemetry.update();
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1000);

            /*
            center.setTargetPosition(1680);
            center.setPower(0.45);
            sleep(1000);
            center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            center.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            center.setPower(0);
            */
            driveBot(10, 10, 0.4, 5.0);
            sleep(2000);
            turn(false, 30);
            sleep(1000);
            arm.setPosition(-0.7);
            sleep(2000);
            claw.setPosition(0.7);
            sleep(2000);
            arm.setPosition(0.7);
            sleep(2000);
            driveBot(-10, -10, 0.4, 5.0);
            sleep(2000);

/*
            driveBot(-19.0,19.0,00.3,5.0);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1000);
*/
            //turn(true, 90);
            sleep(1000);

            // Arm down, claw
            arm.setPosition(1);
            sleep(1000);

        }
        // End RingCount 1

        if (ringCount == 0) {

            driveBot(15.4,15.4,0.4,5.0);
            telemetry.addData("status","first run to position called" );
            telemetry.addData("status", left.getMode() );
            telemetry.addData("status","left motor,  %7d", left.getCurrentPosition() );
            telemetry.addData("status","right motor,  %7d", right.getCurrentPosition() );
            telemetry.update();
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1000);

            /*
            center.setTargetPosition(1680);
            center.setPower(0.45);
            sleep(1000);
            center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            center.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            center.setPower(0);
            */

            turn(false, 85);
            //launcher.setPower(0.78);
            sleep(1000);
            driveBot(15,15,0.4,5.0);
            sleep(1000);
            arm.setPosition(-0.7);
            sleep(2000);
            claw.setPosition(0.7);
/*
            driveBot(-19.0,19.0,00.3,5.0);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1000);
*/
            //turn(true, 90);
            sleep(1000);

            // Arm down, claw
            arm.setPosition(1);
            sleep(1000);
            driveBot(-18,-18,0.4,5.0);

        }
        // End RingCount 0

        if (ringCount == 4) {
            driveBot(-4, 4, 0.4, 5.0);
            //launcher.setPower(0.78);
            sleep(500);
            driveBot(60,60,0.8,5.0);
            telemetry.addData("status","first run to position called" );
            telemetry.addData("status", left.getMode() );
            telemetry.addData("status","left motor,  %7d", left.getCurrentPosition() );
            telemetry.addData("status","right motor,  %7d", right.getCurrentPosition() );
            telemetry.update();
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(200);

            /*
            center.setTargetPosition(1680);
            center.setPower(0.45);
            sleep(1000);
            center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            center.setPower(0);
            */

            //turn(false, 60);
            arm.setPosition(-0.7);
            sleep(1);
            claw.setPosition(0.7);
/*
            driveBot(-19.0,19.0,00.3,5.0);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1000);
*/
            //turn(true, 90);
            sleep(1000);

            // Arm down, claw
            arm.setPosition(1);
            //turn(true, 60);
            sleep(500);
            driveBot(-50,-50,0.8,5.0);

        }
        //End RingCount 4
    }
    // END runOpMode()




    /**********************************************
     * BEGIN METHODS driveDistance, driveBot
     *********************************************/

    /**
     * Drive Distance Method
     * Calculate distance from CM to encoder counts
     */
    public static double driveDistance(double distance)
    {
        double drive  = (ENCODER_CLICKS/ WHEEL_CIRC);
        int outputClicks= (int)Math.floor(drive * distance);
        return outputClicks;
    }
    // END driveDistance() method


    /**
     * Drive Method
     * Left & Right travel distance in CM +/-, power to both wheels, timeout in seconds
     */
    public void driveBot(double distanceInCMleft, double distanceInCMright, double power, double timeoutS)
    {
        telemetry.addData("status","encoder reset");
        telemetry.update();
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int rightTarget;
        int leftTarget;

        if(opModeIsActive())
        {
            telemetry.addData("status","getEncoderClicks");
            telemetry.update();

            rightTarget = (int) driveDistance(distanceInCMright);
            leftTarget = (int) driveDistance(distanceInCMleft);

            right.setTargetPosition(rightTarget);
            left.setTargetPosition(leftTarget);

            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            right.setPower(power);
            left.setPower(power);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (left.isBusy() && right.isBusy()))
            {
                telemetry.addData("Path1", "leftTarget, rightTarget" );
                telemetry.update();
            }
            left.setPower(0);
            right.setPower(0);

            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    // END driveBot method


    /**
     * Turn left with IMU
     * Left travel distance in degrees  +/-, power to both wheels, timeout in seconds
     */
    public void turnLeft()
    {

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //MOVE RIGHT
        left.setPower(-0.2);
        right.setPower(0.2);

        // Continue until robot yaws right by 90 degrees or stop is pressed on Driver Station.
        sleep(1000);
        //turns to the right; 90, 180, negative, -90, 0
        while ( !(Yaw_Angle <= -85 || isStopRequested()) ) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            telemetry.update();
        }
        // We're done. Turn off motors
        left.setPower(0);
        right.setPower(0);
        // Pause so final telemetry is displayed.
        sleep(1000);
    }
    // END turnLeft


    /**
     * Turn right with IMU
     * Left & Right travel distance in degrees +/-, power to both wheels, timeout in seconds
     */
    public void turnRight()
    {

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //MOVE RIGHT
        left.setPower(0.2);
        right.setPower(-0.2);

        // Continue until robot yaws right by 90 degrees or stop is pressed on Driver Station.
        sleep(1000);

        //turns to the right; 90, 180, negative, -90, 0
        while ( Yaw_Angle <= 85 && opModeIsActive() ) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            telemetry.update();
        }
        // We're done. Turn off motors
        left.setPower(0);
        right.setPower(0);
        // Pause so final telemetry is displayed.
        sleep(1000);
    }
    // END turnRight method


    /**
     * Turn  with IMU
     * true for right, degrees are for the direction you're going in
     * Left & Right travel distance in degrees +/-, power to both wheels, timeout in seconds
     */
    public void turn(boolean dir, int degrees)
    {

        // Ensure motors are not in run to position or stop and reset position
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(!dir)
        {
            //MOVE LEFT
            left.setPower(-0.2);
            right.setPower(0.2);
            degrees = (int)(degrees * 0.94444444444);

            //turns to the right; 90, 180, negative, -90, 0
            sleep(1000);
            while ( !(Yaw_Angle <= (-1*degrees) || isStopRequested()) ) {
                // Update Yaw-Angle variable with current yaw.
                Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                // Report yaw orientation to Driver Station.
                telemetry.addData("Yaw value", Yaw_Angle);
                telemetry.update();
            }
        }
        else
        {
            //MOVE RIGHT
            left.setPower(0.2);
            right.setPower(-0.2);
            degrees = (int)(degrees * 0.94444444444);

            sleep(1000);
            while ( !(Yaw_Angle >= degrees || isStopRequested()) ) {
                // Update Yaw-Angle variable with current yaw.
                Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                // Report yaw orientation to Driver Station.
                telemetry.addData("Yaw value", Yaw_Angle);
                telemetry.update();
            }
        }
        // We're done. Turn off motors
        left.setPower(0);
        right.setPower(0);
        // Pause so final telemetry is displayed.
        sleep(1000);
    }
    // END turn method


    /**
     * IMU Calibration Method
     * Checks IMU calibration and returns telementry
     * If IMU is NOT calibrated, run the calibration opMode
     */
    private boolean IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", imu.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", imu.getSystemStatus().toString());
        return imu.isGyroCalibrated();
    }
}