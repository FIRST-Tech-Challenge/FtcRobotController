package org.firstinspires.ftc.team13580;

/*
This file defines a Java class that performs all the setup and configuration for our robot's hardware (motors and
sensors). It has 5 motors (left_front_drive, left_back_drive, right_front_drive, right_back_drive) and to servos
(left_hand and right_hand).

This one file/class is used by ALL of your OpModes without having to cut & paste the code each time.

Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the
class, rather than accessing the internal hardware directly. This is why the objects are declared "private."
 */

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class RobotHardware {

    // Declare the OpMode members.
    private LinearOpMode myOpMode = null; // gains access to the methods in the calling OpMode.

    //Define motors and servo objects (make them private so they can't be accessed externally)
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor upDown = null;
    public DcMotor hang=null;
    //private Servo wrist=null;
    //private Servo wheel=null;
    public DcMotor elbowHang=null;
    public DcMotor spoolie=null;
    public Servo leftHand = null;
    public Servo wrist = null;
    public Servo sweep= null;

    //upDown motor configuration to use encoder
    public final double ARM_TICKS_PER_DEGREE= 28
            * 250047.0/4913.0
            * 100.0/ 20.0
            * 1/ 360.0;

    public final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    public final double ARM_SPECIMEN              = 8*ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE                 = 98 * ARM_TICKS_PER_DEGREE;
    public final double ARM_CLEAR_BARRIER         = 20 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SPECIMEN        = 83 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SAMPLE_IN_LOW   = 107 * ARM_TICKS_PER_DEGREE;
    public final double ARM_ATTACH_HANGING_HOOK   = 130 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SECURE_SPECIMEN       = 190  * ARM_TICKS_PER_DEGREE;
    public final double ARM_COLLECT               = 28 * ARM_TICKS_PER_DEGREE;
    public final double FUDGE_FACTOR              = 15 * ARM_TICKS_PER_DEGREE;
    public final double SPOOLIE_COLLAPSED= 0;
    public final double SPOOLIE_UP_TO_HANG= 15*COUNTS_PER_INCH;

    public double armPosition= (int)ARM_COLLAPSED_INTO_ROBOT;
    public int targetArmPosition;
    public final double HANG_COLLAPSED_INTO_ROBOT =17*ARM_TICKS_PER_DEGREE;
    public final double HANG_UP = 180 * ARM_TICKS_PER_DEGREE;
    public IMU imu = null; // Universal IMU interface

    public ElapsedTime runtime = new ElapsedTime();

    public double drivePower=0;
    public double strafePower=0;
    public double turnPower=0;

    public double leftFrontPower = 0;
    public double leftBackPower = 0;
    public double rightFrontPower = 0;
    public double rightBackPower = 0;


    // Define the drive constants. Make them public so they CAN be used by the calling OpMOde
    public static final double MID_SERVO = 0.3;
    public static final double ZERO_SERVO = 0.025;
    public static final double RETRACT_SERVO=0.5;
    public static final double HAND_SPEED = 0.2;
    public static final double ARM_UP_POWER = 0.5;
    public static final double ARM_DOWN_POWER = -0.45;
    public static final double SPOOLIE_UP_POWER = 1;
    public static final double SPOOLIE_DOWN_POWER = -0.8;

    //Define encoder constants
    public static final double COUNTS_PER_MOTOR_REV = 537.7;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES=4.0;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES*3.1415);
    public static final double DRIVE_SPEED =  0.6;
    public static final double TURN_SPEED= 0.5;

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public double armTarget=0;

    //Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {myOpMode = opmode;}

    /**
     * initialize all the robot's hardware. This method must be called ONCE when the OpMOde is Initialized.
     * <p>
     * All the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init (){
        // Define and initialize Motors (note: we need to use reference to actual OpMode)
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        spoolie= myOpMode.hardwareMap.get(DcMotor.class, "spoolie");
        hang= myOpMode.hardwareMap.get(DcMotor.class, "hang");
        upDown= myOpMode.hardwareMap.get(DcMotor.class, "up_down");
        elbowHang= myOpMode.hardwareMap.get(DcMotor.class, "elbow_hang");
        /* To drive forward, our robot need the motors on one side to be reversed, because the axles point in opposite
         direction.
         Pushing the left stick forward MUST make robot go forward. So we will adjust these lines if necessary, after
         the first test drive.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        elbowHang.setDirection(DcMotor.Direction.REVERSE);

        leftHand= myOpMode.hardwareMap.get(Servo.class, "left_hand");
        wrist=myOpMode.hardwareMap.get(Servo.class, "wrist");
        leftHand.setPosition(MID_SERVO);
        wrist.setPosition(ZERO_SERVO);

        sweep=myOpMode.hardwareMap.get(Servo.class, "sweep");
        sweep.setPosition(RETRACT_SERVO);


        imu= myOpMode.hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters( new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        upDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spoolie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbowHang.setTargetPosition(0);
        elbowHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        upDown.setTargetPosition(0);
        upDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //encoder initialization for the wheels
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested robot motion;
     * Drive (axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial     Fwd/Rev driving power  (-1.0 to 1.0) +ve is forward
     * @param lateral   Left/Right driving power (-1.0 to 1.0) +ve is right
     * @param yaw       Right/Left turning power (-1.o to 1.0) +ve is CW
     */


    public void driveRobotCentric(double axial, double lateral, double yaw) {
        double max;

        // Combine drive and Turn for blended motion.
        leftFrontPower = axial + lateral + yaw;
        leftBackPower = axial - lateral + yaw;
        rightFrontPower = axial - lateral -yaw;
        rightBackPower = axial + lateral -yaw;

        // Scale the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(leftFrontPower) , Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max= Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0){
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        //Use existing function to drive both wheels.
        setDrivePower (leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested robot motion;
     * Drive (axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param drive     Fwd/Rev driving power  (-1.0 to 1.0) +ve is forward
     * @param strafe   Left/Right driving power (-1.0 to 1.0) +ve is right
     * @param turn       Right/Left turning power (-1.o to 1.0) +ve is CW
     */

    public void driveFieldCentric(double drive, double strafe, double turn) {
        //make the parameters store the value assigned when calling the method
        drivePower=drive;
        strafePower=strafe;
        turnPower=turn;

        double max;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double strafeRotation= strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double driveRotation = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);


        // Combine drive and Turn for blended motion.
        leftFrontPower = driveRotation + strafeRotation + turn;
        leftBackPower = driveRotation - strafeRotation + turn;
        rightFrontPower = driveRotation - strafeRotation - turn;
        rightBackPower = driveRotation + strafeRotation -turn;

        // Scale the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(leftFrontPower) , Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max= Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0){
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        //Use existing function to drive both wheels.
        setDrivePower (leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);


    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftFrontWheel        Fwd/Rev driving power {-1.0 to 1.0) +ve is forward
     * @param leftBackWheel
     * @param rightFrontWheel
     * @param rightBackWheel       Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */

    public void setDrivePower(double leftFrontWheel, double leftBackWheel, double rightFrontWheel, double rightBackWheel) {
        //Output the values of the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        rightBackDrive.setPower(rightBackWheel);
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power   (-1.0 to 1.0)
     */

    public void setSpooliePower (double power) { spoolie.setPower(power);}

    public void setHangPower (double power) { hang.setPower(power);}

    //Sets the power used to move the elbow(Motor on the arm not the spoolie) of the robot
    //public void setUpDownPower (double power) { upDown.setPower(power);}

    /**
     * Send the twon hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
   // public void wristPosition(double offset){
   //     offset = Range.clip(offset, -0.5, 0.5);
   //     wrist.setPosition(MID_SERVO+offset);
   // }

   // public void wheelPosition(double offset){
   //     offset = Range.clip(offset, -0.5, 0.5);
   //     wheel.setPosition(MID_SERVO+offset);
   // }
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5 );
        leftHand.setPosition(MID_SERVO + offset);
        //rightHand.setPosition(MID_SERVO - offset);

    }

    public void encoderArm( double armDegree, double timeoutS ){
        int newArmTarget;
        int armPositionFudgeFactor;
        if(myOpMode.opModeIsActive()){
            newArmTarget= upDown.getCurrentPosition()+(int)(armDegree * ARM_TICKS_PER_DEGREE);
            armTarget=(int)(newArmTarget+FUDGE_FACTOR);
            upDown.setTargetPosition((int)(newArmTarget+FUDGE_FACTOR));

            upDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            upDown.setPower(Math.abs(1500));

           while(myOpMode.opModeIsActive()&&(runtime.seconds()<timeoutS) && (upDown.isBusy())){
               myOpMode.telemetry.addData("Running to",  " %7d ", newArmTarget);
               myOpMode.telemetry.addData("Currently at",  " at %7d ",
                       upDown.getCurrentPosition());
               myOpMode.telemetry.update();
           }

            upDown.setPower(0);
            upDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }

    public void encoderArmDrive(double speed,double leftFrontInches, double leftBackInches,
                                double rightFrontInches, double rightBackInches,double armDegree,
                                double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;
        int newArmTarget;

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            newArmTarget= upDown.getCurrentPosition()+(int)(armDegree * ARM_TICKS_PER_DEGREE);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);
            upDown.setTargetPosition((int) armTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            upDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setDrivePower(Math.abs(speed),Math.abs(speed),Math.abs(speed),Math.abs(speed));
            upDown.setPower(Math.abs(1500));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy()&& rightFrontDrive.isBusy() && rightBackDrive.isBusy())&&(upDown.isBusy())) {

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to",  " %7d :%7d :%7d :%7d :%7d", newLeftFrontTarget,  newLeftBackTarget, newRightFrontTarget,  newRightBackTarget, newArmTarget);
                myOpMode.telemetry.addData("Starting at",  " at %7d :%7d :%7d :%7d :%7d",
                        leftFrontDrive.getCurrentPosition(),leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(), upDown.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            upDown.setPower(0);


            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            upDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //newArmTarget= upDown.getCurrentPosition()+(int)(armDegree * ARM_TICKS_PER_DEGREE);

            //upDown.setTargetPosition((int)(newArmTarget+FUDGE_FACTOR));

            //upDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //runtime.reset();
            //upDown.setPower(Math.abs(1500));

           // while(myOpMode.opModeIsActive() && (runtime.seconds()<timeoutS)&&(upDown.isBusy())){
                //myOpMode.telemetry.addData("Running to",  " %7d ", newArmTarget);
                //myOpMode.telemetry.addData("Currently at",  " at %7d ",
                        //upDown.getCurrentPosition());
                //myOpMode.telemetry.update();
            //}

            //upDown.setPower(0);
            //upDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void encoderDrive(double speed,
                             double leftFrontInches, double leftBackInches,
                             double rightFrontInches, double rightBackInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setDrivePower(Math.abs(speed),Math.abs(speed),Math.abs(speed),Math.abs(speed));
            upDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            upDown.setTargetPosition((int) armTarget);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy()&& rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newLeftFrontTarget,  newLeftBackTarget, newRightFrontTarget,  newRightBackTarget);
                myOpMode.telemetry.addData("Starting at",  " at %7d :%7d :%7d :%7d",
                        leftFrontDrive.getCurrentPosition(),leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }
    public void encoderSpoolie(double speed,
                             double spoolieInches,
                             double timeoutS) {
        int newSpoolieTarget;

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newSpoolieTarget = spoolie.getCurrentPosition() + (int)(spoolieInches * COUNTS_PER_INCH);

            spoolie.setTargetPosition(newSpoolieTarget);

            // Turn On RUN_TO_POSITION
            spoolie.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setSpooliePower(Math.abs(2100));
            spoolie.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            spoolie.setTargetPosition(spoolie.getCurrentPosition());

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (spoolie.isBusy())){

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to",  " %7d", newSpoolieTarget);
                myOpMode.telemetry.addData("Starting at",  " at %7d",
                        spoolie.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            // Stop all motion;
            spoolie.setPower(0);


            // Turn off RUN_TO_POSITION
            spoolie.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

}
