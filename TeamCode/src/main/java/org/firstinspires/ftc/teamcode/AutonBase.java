package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.teamcode.hardware.Light;
//import org.firstinspires.ftc.teamcode.hardware.LightMode;
//import org.firstinspires.ftc.teamcode.hardware.RobotControlArm;
//import org.firstinspires.ftc.teamcode.hardware.RobotControlFlipperMotor;
//import org.firstinspires.ftc.teamcode.hardware.RobotControlGripperServos;
//import org.firstinspires.ftc.teamcode.hardware.RobotControlLights;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.NectarShooter;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.camera;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AutonBase extends LinearOpMode {

    /* Declare OpMode members. */
    protected RobotHardwareMap theHardwareMap;
    Intake intake;

    //Outtake outtake;
    camera Camera;

    private IMU imu         = null;

    private double targetHeading = 0;
    private double headingOffset = 0;
    private double robotHeading = 0;
    private double headingError = 0;

    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.06;
    static final double     HEADING_THRESHOLD       = 1.0 ;

    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.8 ; //3.77953     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public Shooter getPollenShooter() {
        return pollenShooter;
    }
    private Shooter pollenShooter;

    public NectarShooter getNectarShooter() {
        return nectarShooter;
    }

    private NectarShooter nectarShooter;

//    RobotCameraHandler robotCameraHandler;
//    RobotControlLights lights;

    public Intake getIntake() {
        return intake;
    }

    public void initialize() {
        theHardwareMap  = new RobotHardwareMap(hardwareMap, this);
//        robotCameraHandler = new RobotCameraHandler(theHardwareMap, this);
//        lights = new RobotControlLights(theHardwareMap, this);

        theHardwareMap.initialize();
//        robotCameraHandler.initialize();

        imu = theHardwareMap.chImu;

        theHardwareMap.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        theHardwareMap.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        theHardwareMap.frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        theHardwareMap.frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        theHardwareMap.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        theHardwareMap.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        theHardwareMap.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        theHardwareMap.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = new Intake(theHardwareMap, this);
        intake.initialize();

        //outtake = new Outtake(theHardwareMap, this);
        //outtake.initialize();
        pollenShooter = new Shooter(theHardwareMap,this);
        pollenShooter.init();
//        nectarShooter = new NectarShooter(theHardwareMap,this);
//        nectarShooter.init();
        //Camera = new camera(theHardwareMap, this);

        resetHeading();
    }

    public void encoderDrive (double speed, double inches, double timeoutS) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = theHardwareMap.frontLeftMotor.getCurrentPosition() + (int)(inches * RobotGlobalSettings.COUNTS_PER_INCH);
            newBackLeftTarget = theHardwareMap.backLeftMotor.getCurrentPosition() + (int)(inches * RobotGlobalSettings.COUNTS_PER_INCH);
            newFrontRightTarget = theHardwareMap.frontRightMotor.getCurrentPosition() + (int)(inches * RobotGlobalSettings.COUNTS_PER_INCH);
            newBackRightTarget = theHardwareMap.backRightMotor.getCurrentPosition() + (int)(inches * RobotGlobalSettings.COUNTS_PER_INCH);

            theHardwareMap.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            theHardwareMap.backLeftMotor.setTargetPosition(newBackLeftTarget);
            theHardwareMap.frontRightMotor.setTargetPosition(newFrontRightTarget);
            theHardwareMap.backRightMotor.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            theHardwareMap.frontLeftMotor.setPower(Math.abs(speed));
            theHardwareMap.backLeftMotor.setPower(Math.abs(speed));
            theHardwareMap.frontRightMotor.setPower(Math.abs(speed));
            theHardwareMap.backRightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (theHardwareMap.frontLeftMotor.isBusy() && theHardwareMap.backLeftMotor.isBusy()
                            && theHardwareMap.frontRightMotor.isBusy() && theHardwareMap.backRightMotor.isBusy())) {
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d",
                        newFrontLeftTarget,  newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                        theHardwareMap.frontLeftMotor.getCurrentPosition(), theHardwareMap.backLeftMotor.getCurrentPosition(),
                        theHardwareMap.frontRightMotor.getCurrentPosition(), theHardwareMap.backRightMotor.getCurrentPosition());
                telemetry.update();
                //outtake.ControlMotorSpeed();
            }

            theHardwareMap.frontLeftMotor.setPower(0);
            theHardwareMap.backLeftMotor.setPower(0);
            theHardwareMap.frontRightMotor.setPower(0);
            theHardwareMap.backRightMotor.setPower(0);

            theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    static final double COUNTS_PER_DEGREE = 9.8;

    // Fast, uncalibrated turn using only the wheel encoders (no IMU).
// Positive degrees = turn right (same as imuTurn): left wheels forward, right wheels back.
    public void encoderTurn(double speed, double degrees) {
        if (!opModeIsActive()) return;

        int counts = (int) (degrees * COUNTS_PER_DEGREE);

        int frontLeftTarget  = theHardwareMap.frontLeftMotor.getCurrentPosition()  + counts;
        int backLeftTarget   = theHardwareMap.backLeftMotor.getCurrentPosition()   + counts;
        int frontRightTarget = theHardwareMap.frontRightMotor.getCurrentPosition() - counts;
        int backRightTarget  = theHardwareMap.backRightMotor.getCurrentPosition()  - counts;

        theHardwareMap.frontLeftMotor.setTargetPosition(frontLeftTarget);
        theHardwareMap.backLeftMotor.setTargetPosition(backLeftTarget);
        theHardwareMap.frontRightMotor.setTargetPosition(frontRightTarget);
        theHardwareMap.backRightMotor.setTargetPosition(backRightTarget);

        theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        theHardwareMap.frontLeftMotor.setPower(Math.abs(speed));
        theHardwareMap.backLeftMotor.setPower(Math.abs(speed));
        theHardwareMap.frontRightMotor.setPower(Math.abs(speed));
        theHardwareMap.backRightMotor.setPower(Math.abs(speed));

        // wait until all four motors reach their targets
        while (opModeIsActive() &&
                (theHardwareMap.frontLeftMotor.isBusy() && theHardwareMap.backLeftMotor.isBusy()
                        && theHardwareMap.frontRightMotor.isBusy() && theHardwareMap.backRightMotor.isBusy())) {
            // just waiting
        }

        // stop and go back to normal encoder mode
        theHardwareMap.frontLeftMotor.setPower(0);
        theHardwareMap.backLeftMotor.setPower(0);
        theHardwareMap.frontRightMotor.setPower(0);
        theHardwareMap.backRightMotor.setPower(0);

        theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void imuDrive(double maxDriveSpeed,
                         double distance,
                         double degrees) {

        int leftRearTarget;
        int rightRearTarget;
        int leftFrontTarget;
        int rightFrontTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftRearTarget = theHardwareMap.backLeftMotor.getCurrentPosition() + moveCounts;
            rightRearTarget = theHardwareMap.backRightMotor.getCurrentPosition() + moveCounts;
            leftFrontTarget = theHardwareMap.frontLeftMotor.getCurrentPosition() + moveCounts;
            rightFrontTarget = theHardwareMap.frontRightMotor.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            theHardwareMap.backLeftMotor.setTargetPosition(leftRearTarget);
            theHardwareMap.backRightMotor.setTargetPosition(rightRearTarget);
            theHardwareMap.frontLeftMotor.setTargetPosition(leftFrontTarget);
            theHardwareMap.frontRightMotor.setTargetPosition(rightFrontTarget);

            theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (theHardwareMap.backLeftMotor.isBusy() && theHardwareMap.backRightMotor.isBusy()&&
                            theHardwareMap.frontLeftMotor.isBusy() && theHardwareMap.frontRightMotor.isBusy())) {
                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(degrees, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);
                //outtake.ControlMotorSpeed();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            imuTurn(1, -degrees);   // correct back to the heading we were holding
        }
    }




    public void encoderStrafe(double speed, double strafeDistance, double timeoutS){
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        if (opModeIsActive()){

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = theHardwareMap.frontLeftMotor.getCurrentPosition() + (int)(strafeDistance * COUNTS_PER_INCH);
            newBackLeftTarget = theHardwareMap.backLeftMotor.getCurrentPosition() + (int)(-strafeDistance * COUNTS_PER_INCH);
            newFrontRightTarget = theHardwareMap.frontRightMotor.getCurrentPosition() + (int)(-strafeDistance * COUNTS_PER_INCH);
            newBackRightTarget = theHardwareMap.backRightMotor.getCurrentPosition() + (int)(strafeDistance * COUNTS_PER_INCH);

            theHardwareMap.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            theHardwareMap.backLeftMotor.setTargetPosition(newBackLeftTarget);
            theHardwareMap.frontRightMotor.setTargetPosition(newFrontRightTarget);
            theHardwareMap.backRightMotor.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            theHardwareMap.frontLeftMotor.setPower(Math.abs(speed));
            theHardwareMap.backLeftMotor.setPower(Math.abs(speed));
            theHardwareMap.frontRightMotor.setPower(Math.abs(speed));
            theHardwareMap.backRightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (theHardwareMap.frontLeftMotor.isBusy() && theHardwareMap.backLeftMotor.isBusy()
                            && theHardwareMap.frontRightMotor.isBusy() && theHardwareMap.backRightMotor.isBusy())) {


                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d",
                        newFrontLeftTarget,  newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                        theHardwareMap.frontLeftMotor.getCurrentPosition(), theHardwareMap.backLeftMotor.getCurrentPosition(),
                        theHardwareMap.frontRightMotor.getCurrentPosition(), theHardwareMap.backRightMotor.getCurrentPosition());
                telemetry.update();
                //outtake.ControlMotorSpeed();
            }

            // Stop all motion;
            theHardwareMap.frontLeftMotor.setPower(0);
            theHardwareMap.backLeftMotor.setPower(0);
            theHardwareMap.frontRightMotor.setPower(0);
            theHardwareMap.backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }





    public void imuStrafe(double speed, double strafeDistance, double degrees, double timeoutS){
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        if (opModeIsActive()){

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = theHardwareMap.frontLeftMotor.getCurrentPosition() + (int)(strafeDistance * COUNTS_PER_INCH);
            newBackLeftTarget = theHardwareMap.backLeftMotor.getCurrentPosition() + (int)(-strafeDistance * COUNTS_PER_INCH);
            newFrontRightTarget = theHardwareMap.frontRightMotor.getCurrentPosition() + (int)(-strafeDistance * COUNTS_PER_INCH);
            newBackRightTarget = theHardwareMap.backRightMotor.getCurrentPosition() + (int)(strafeDistance * COUNTS_PER_INCH);

            theHardwareMap.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            theHardwareMap.backLeftMotor.setTargetPosition(newBackLeftTarget);
            theHardwareMap.frontRightMotor.setTargetPosition(newFrontRightTarget);
            theHardwareMap.backRightMotor.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            theHardwareMap.frontLeftMotor.setPower(Math.abs(speed));
            theHardwareMap.backLeftMotor.setPower(Math.abs(speed));
            theHardwareMap.frontRightMotor.setPower(Math.abs(speed));
            theHardwareMap.backRightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (theHardwareMap.frontLeftMotor.isBusy() && theHardwareMap.backLeftMotor.isBusy()
                            && theHardwareMap.frontRightMotor.isBusy() && theHardwareMap.backRightMotor.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(degrees, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (strafeDistance < 0)
                    turnSpeed *= -1.0;

                theHardwareMap.frontLeftMotor.setPower(Math.abs(speed) - turnSpeed);
                theHardwareMap.frontRightMotor.setPower(Math.abs(speed) - turnSpeed);
                theHardwareMap.backLeftMotor.setPower(Math.abs(speed) + turnSpeed);
                theHardwareMap.backRightMotor.setPower(Math.abs(speed) + turnSpeed);
                //outtake.ControlMotorSpeed();






                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d",
                        newFrontLeftTarget,  newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                        theHardwareMap.frontLeftMotor.getCurrentPosition(), theHardwareMap.backLeftMotor.getCurrentPosition(),
                        theHardwareMap.frontRightMotor.getCurrentPosition(), theHardwareMap.backRightMotor.getCurrentPosition());
                telemetry.update();
                //outtake.ControlMotorSpeed();
            }


            // Stop all motion;
            theHardwareMap.frontLeftMotor.setPower(0);
            theHardwareMap.backLeftMotor.setPower(0);
            theHardwareMap.frontRightMotor.setPower(0);
            theHardwareMap.backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            imuTurn(1, -degrees);   // turn back to the heading we were holding

            sleep(250);   // optional pause after each move.
        }
    }


    public void imuTurn(double maxTurnSpeed, double degrees) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(-degrees, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(-degrees, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
            //outtake.ControlMotorSpeed();
        }

        // Stop all motion;
        moveRobot(0, 0);
        resetHeading();
    }

    public void aprilTagOutakeSpeedAdjustAndAlignment(){
        double turnSpeed = 1;
        Camera.loop();
        //outtake.setSpeed(Camera.outtakespeedfordistance(outtake.getspeed()));
/*        while (opModeIsActive() && (turnSpeed > 0.3)) {
            Camera.loop();
            turnSpeed = Camera.Robotallignwithgoal(0);
            moveRobot(0, turnSpeed);
            outtake.ControlMotorSpeed();
        }


        // Stop all motion;
        moveRobot(0, 0);
        resetHeading();*/
    }

    public void transferAndLaunchArtifacts() {
        //aprilTagOutakeSpeedAdjustAndAlignment();
        //outtake.blockingShoot();
        //outtake.outtakeoff();
    }

    private void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        theHardwareMap.backLeftMotor.setPower(leftSpeed);
        theHardwareMap.backRightMotor.setPower(rightSpeed);
        theHardwareMap.frontLeftMotor.setPower(leftSpeed);
        theHardwareMap.frontRightMotor.setPower(rightSpeed);
    }

    private double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    private double getRawHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yaw   = orientation.getYaw(AngleUnit.DEGREES);
        double pitch   = orientation.getPitch(AngleUnit.DEGREES);
        double roll   = orientation.getRoll(AngleUnit.DEGREES);
        Log.println(Log.INFO, "IMU Yaw: ", String.valueOf(yaw));
        Log.println(Log.INFO, "IMU Pitch: ", String.valueOf(pitch));
        Log.println(Log.INFO, "IMU Roll: ", String.valueOf(roll));
        return yaw;
    }

    private void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        imu.resetYaw();
        headingOffset = getRawHeading();
        Log.println(Log.INFO, "Heading Offset", String.valueOf(headingOffset));
        robotHeading = 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}