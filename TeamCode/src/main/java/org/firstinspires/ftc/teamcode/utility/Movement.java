package org.firstinspires.ftc.teamcode.utility;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**refor
 * This class contains methods to control drive base movement
 */
public class Movement {

    private final VisionSystem visionProcessor;
    public int motor_ticks;

    public static enum Direction {
        FRONT,
        BACK,
        LEFT,
        RIGHT
    }
    private DcMotorEx lfDrive;
    private DcMotorEx rfDrive;
    private DcMotorEx lbDrive;
    private DcMotorEx rbDrive;
    private IMU imu;
    /**
     * Setup Kinematics and Odometry objects from FTClib
     */
    MecanumDriveKinematics kinematics;
    MecanumDriveOdometry odometry;
    ElapsedTime odometryTimer;
    MecanumDriveWheelSpeeds odometrySpeeds;
    private RevBlinkinLedDriver blinkinLED;
    private Telemetry telemetry;

    // Tracks if it's quicker to turn right or left
    double turnError = 0;

    double moveStartDirection = 0.0;
    int alignStage = 0;

    double tagRange = 15;
    double tagBearing = 0;
    boolean tagDetected = false;
    double aprilTagCurrentX = -2;
    double aprilTagCurrentY = 15;
    boolean aprilTagAligned = false;
    boolean pose2dAligned = false;

    // Setup PID controllers for Pose2d motion.
    PIDController yawPID;
    PIDController axialPID;
    PIDController lateralPID;
    double axial = 0;
    double lateral = 0;
    double yaw = 0;

    /**
     * Pulls in information about the motors that is determined during initialization and makes
     * that information accessible to the rest of the class.
     *
     * @param leftFrontDrive       the front left wheels motor,
     * @param rightFrontDrive      the front right wheels motor,
     * @param leftBackDrive        the back left wheels motor,
     * @param rightBackDrive       the back right wheels motor,
     * @param imu1                 the ControlHub gyro,
     * @param blinkinLED1          the BlinkinLED,
     * @param telemetry1           telemetry for the Drive Station
     */
    public Movement(DcMotorEx leftFrontDrive,
                    DcMotorEx rightFrontDrive,
                    DcMotorEx leftBackDrive,
                    DcMotorEx rightBackDrive,
                    IMU imu1,
                    RevBlinkinLedDriver blinkinLED1,
                    MecanumDriveOdometry odometry1,
                    MecanumDriveKinematics kinematics1,
                    ElapsedTime odometryTimer1,
                    MecanumDriveWheelSpeeds odometrySpeeds1,
                    Telemetry telemetry1,
                    VisionSystem vProc){
        lfDrive = leftFrontDrive;
        rfDrive = rightFrontDrive;
        lbDrive = leftBackDrive;
        rbDrive = rightBackDrive;
        imu = imu1;
        blinkinLED = blinkinLED1;
        kinematics = kinematics1;
        odometry = odometry1;
        odometryTimer = odometryTimer1;
        odometrySpeeds = odometrySpeeds1;
        telemetry = telemetry1;
        visionProcessor = vProc;

        initOdometry();
        yawPID = new PIDController((1.0/45.0), 0.0015, 0.0025);
        yawPID.reset();
        axialPID = new PIDController(2.5, 0.005, 0.02);
        axialPID.reset();
        lateralPID = new PIDController(7, 0.005, 0.02);
        lateralPID.reset();
    }

    public void initOdometry() {
        // Setup the 2d translation for GGE as coordinates of each motor, relative to the center of GGE.
        // in Meters - translated from inches as inches * 2.54 / 100
        Translation2d lfMotorMeters = new Translation2d(-(6 * 2.54 / 100.0), (3.5 * 2.54 / 100.0));
        Translation2d rfMotorMeters = new Translation2d((6 * 2.54 / 100.0), (5 * 2.54 / 100.0));
        Translation2d lbMotorMeters = new Translation2d(-(6 * 2.54 / 100.0), -(3.5 * 2.54 / 100.0));
        Translation2d rbMotorMeters = new Translation2d((6 * 2.54 / 100.0), -(5 * 2.54 / 100.0));

        // Create Mecanum Kinematics
        kinematics = new MecanumDriveKinematics(lfMotorMeters, rfMotorMeters, lbMotorMeters, rbMotorMeters);

        // Create Mecanum Odometry
        odometry = new MecanumDriveOdometry(kinematics, new Rotation2d(0.0));

        //Todo: Need to take the position information passed into GGE_Odometry set the starting position
        odometryTimer = new ElapsedTime();
        odometryTimer.reset();
        imu.resetYaw();
        // Initialize the odometry to the start position of the robot.
        // TODO* - this will only presently work right from the blue field left start position and will need stored values from each location
        odometry.resetPosition(new Pose2d(0.25, 2.2, new Rotation2d(Math.toRadians(0.0))),
                new Rotation2d (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

    }

    public MecanumDriveOdometry getOdometry() {
        return odometry;
    }

    /**
     * Resets all wheel motor encoder positions to 0
     */
    private void initMovement() {
        motor_ticks = 0;

        moveStartDirection = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        lfDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        //lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        rfDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        //rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        lbDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        //lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        rbDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        //rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done


    }

    /**
     * Setup a method to return the wheel speeds from GGE
     *
     * @return
     */
    public MecanumDriveWheelSpeeds GetWheelSpeeds() {
        MecanumDriveWheelSpeeds speeds = new MecanumDriveWheelSpeeds();
        // 1 entered as a placeholder for a future constant - fix as needed
        // Scalar value obtained by measuring 100cm and dividing out observed values untuned
        speeds.frontLeftMetersPerSecond = (1.0 / 1270.0) * lfDrive.getVelocity();
        speeds.frontRightMetersPerSecond = (1.0 / 1270.0) * rfDrive.getVelocity();
        speeds.rearLeftMetersPerSecond = (1.0 / 1270.0) * lbDrive.getVelocity();
        speeds.rearRightMetersPerSecond = (1.0 / 1270.0) * rbDrive.getVelocity();

        return speeds;
    }

    /**
     * Stops all drive motors.
     */
    public void StopMotors() {
        // Set Powers to 0 for safety and not knowing what they are set to.
        lfDrive.setPower(0);
        rfDrive.setPower(0);
        lbDrive.setPower(0);
        rbDrive.setPower(0);
    }

    /**
     * Moves all wheel motors forward a distance in ticks
     *
     * @param power the power given to the motors
     */
    public void Forward(int ticks, double power) {
        initMovement();
        lfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(power);
        rfDrive.setPower(power);
        lbDrive.setPower(power);
        rbDrive.setPower(power);
        // Hold the start of the next command until this movement is within 30 ticks of its position
        while (abs(lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) > 30) {
            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > moveStartDirection) {
                lfDrive.setPower(power * 1.15);
                lbDrive.setPower(power * 1.15);
            } else {
                lfDrive.setPower(power * 0.85);
                lbDrive.setPower(power * 0.85);
            }
        }
    }

    /**
     * Moves all wheel motors backwards a distance in ticks
     *
     * @param power the power given to the motors
     */
    public void Backwards(int ticks, double power) {
        initMovement();
        lfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(power);
        rfDrive.setPower(power);
        lbDrive.setPower(power);
        rbDrive.setPower(power);
        // Hold the start of the next command until this movement is within 30 ticks of its position
        while (abs(lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) > 30) {
            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > moveStartDirection) {
                lfDrive.setPower(power * 0.85);
                lbDrive.setPower(power * 0.85);
            } else {
                lfDrive.setPower(power * 1.15);
                lbDrive.setPower(power * 1.15);
            }
        }
    }

    /**
     * Moves the front right and the back left motor forwards, and the front left and the back right
     * motors backwards in order to move left a distance in ticks
     *
     * @param power the power given to the motors
     */
    public void Left(int ticks, double power) {
        initMovement();
        lfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(power);
        rfDrive.setPower(power);
        lbDrive.setPower(power);
        rbDrive.setPower(power);
        // Hold the start of the next command until this movement is within 30 ticks of its position
        while (abs(lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) > 30) {
            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > moveStartDirection) {
                lbDrive.setPower(power * 1.15);
                rbDrive.setPower(power * 1.15);
            } else {
                lbDrive.setPower(power * 0.85);
                rbDrive.setPower(power * 0.85);
            }
        }
    }

    /**
     * Moves the front left and the back right motor forwards, and the front right and the back left
     * motors backwards in order to move left a distance in ticks
     *
     * @param power the power given to the motors
     */
    public void Right(int ticks, double power) {
        initMovement();
        lfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(power);
        rfDrive.setPower(power);
        lbDrive.setPower(power);
        rbDrive.setPower(power);
        // Hold the start of the next command until this movement is within 30 ticks of its position
        while (abs(lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) > 30) {
            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > moveStartDirection) {
                lbDrive.setPower(power * 0.85);
                rbDrive.setPower(power * 0.85);
            } else {
                lbDrive.setPower(power * 1.15);
                rbDrive.setPower(power * 1.15);
            }
        }
    }

    /**
     * turns the robot in place a distance in degrees
     *
     * @param degrees - the distance of the rotation in degrees
     */
    public void Rotate(double degrees) {

        double currentDirection = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        turnError = degrees + 2.5; // This works starting with +2.5 degrees (possibly inertia)

        // Through measurement, Gge takes 1500 ticks to complete a full rotation
        // Set the initial stepSize to what we expect the rotation to need
        int stepSize = (int) (turnError / 360 * 1580);

        // Move slowly and in increments
        //Move the right front motor forward
        rfDrive.setTargetPosition(rfDrive.getCurrentPosition() + (stepSize * -1));
        // Move the left front motor backwards
        lfDrive.setTargetPosition(lfDrive.getCurrentPosition() + (stepSize));
        // Move the right back motor forward
        rbDrive.setTargetPosition(rbDrive.getCurrentPosition() + (stepSize * -1));
        // Move the left back motor backward
        lbDrive.setTargetPosition(lbDrive.getCurrentPosition() + (stepSize));

        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rfDrive.setPower(0.5);
        lfDrive.setPower(0.5);
        rbDrive.setPower(0.5);
        lbDrive.setPower(0.5);

        currentDirection = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        turnError = degrees - currentDirection;
        //Closed loop turn.  Stay in the while loop until the desired bering is achieved.
        while (abs(lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) < 10) {
            // continue rotation until at target angle.
            telemetry.addData("Turn Target Direction", lbDrive.getTargetPosition());
            telemetry.addData("Turn Current Direction", lbDrive.getCurrentPosition());
            telemetry.update();
        }
    }

    /**
     * Calculates the turn error and contains it within 180 and -180
     *
     * @param desiredDirection - which direction you want to go to
     * @param currentDirection - your current direction
     */
    public double CalcTurnError(double desiredDirection, double currentDirection) {
        double turnDiff = desiredDirection - currentDirection;
        if (turnDiff < -180) {
            turnDiff = turnDiff + 360;
        } else if (turnDiff > 180) {
            turnDiff = turnDiff - 360;
        }
        return turnDiff;
    }

    public boolean GoToAprilTag(int tagNumber) {
        double aprilTagTargetX = 0;
        // The AprilTag is not centered on the LEFT and RIGHT backdrop zones, adjust X targets
        if (tagNumber == 1 || tagNumber == 4) {
            aprilTagTargetX = 0.5;
        } else if (tagNumber == 3 || tagNumber == 6) {
            aprilTagTargetX = -0.5;
        }
        double aprilTagTargetY = 9;
        double aprilTagTargetAngle = 0;

        // Translate the tagNumber requested to know the angle of the backdrop in robot IMU
        if (tagNumber <= 3) {
            aprilTagTargetAngle = -90;
        } else if (tagNumber > 3) {
            aprilTagTargetAngle = 90;
        }

        double aprilTagCurrentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Scan for April Tag detections and update current values if you find one.
        List<AprilTagDetection> tag = visionProcessor.getDetections();
        if (tag != null) {
            for (int i = 0; i < tag.size(); i++) {
                if (tag.get(i) != null) {
                    if (tag.get(i).id == tagNumber) {
                        aprilTagCurrentX = tag.get(i).ftcPose.x;
                        aprilTagCurrentY = tag.get(i).ftcPose.y;
                        tagRange = tag.get(i).ftcPose.range;
                        tagBearing = tag.get(i).ftcPose.bearing;
                        blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        tagDetected = true;
                    }
                }
            }
        }

        // Update Telemetry with key data
        telemetry.addData("tags found: ", tag.size());
        telemetry.addData("AlignStage: ", alignStage);
        telemetry.addData("Current X: ", aprilTagCurrentX);
        telemetry.addData("Target X: ", aprilTagTargetX);
        telemetry.addData("Current Y: ", aprilTagCurrentY);
        telemetry.addData("Target Y: ", aprilTagTargetY);
        telemetry.addData("Tag Range: ", tagRange);
        telemetry.addData("Tag Bearing: ", tagBearing);
        telemetry.addData("Current Angle: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Target Angle: ", aprilTagTargetAngle);
        telemetry.update();

        // Like the driver control TeleOp, consider the needed axial, lateral and yaw for
        // the motion needed to get to the April Tag.
        // axial from drive is gamepad1.left_stick_y;
        // lateral from drive is -gamepad1.left_stick_x;
        // yaw from drive is -gamepad1.right_stick_x;

        // Stage 0 - Ensure that motor powers are zeroed and switch to RUN_USING_ENCODER mode.
        if (alignStage == 0) {
            // Motors will need to be in RUN_USING_ENCODER for this vs. RUN_TO_POSITION mode
            // Refactor this to the Movement class to make a method to switch motors to run
            // on a defined power level.
            lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Set default to BRAKE mode for control
            lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //
            alignStage = 1;
        }

        axial = -0.10;
        // Assume april tag is not aligned at start.
        aprilTagAligned = false;

        // Square up the robot to the backdrop (from targetAngle above)
        // If the yaw is +, apply -yaw, if the yaw if -, apply +yaw (-right_stick_x in robot mode)
        if (abs(aprilTagTargetAngle - aprilTagCurrentAngle) > 2) {
            yaw = -CalcTurnError(aprilTagTargetAngle, aprilTagCurrentAngle) / 45;
            if (yaw > 0.2) {
                yaw = 0.2;
            } else if (yaw < -0.2) {
                yaw = -0.2;
            }
        } else {
            yaw = 0;
        }

        // Slide laterally to correct for X or right motion
        // If the x distance is > 1 inch off of targetX move left or right accordingly
        // To make the robot go right, reduce the lateral (-left_stick_x in robot mode)
        // To make the robot go left, increase the lateral (-left_stick_x in robot mode)
        //lateral = (targetX - currentY) / 20;
        if (aprilTagTargetX - aprilTagCurrentX > 1) {
            lateral = 0.2;
        } else if (aprilTagTargetX - aprilTagCurrentX < -1) {
            lateral = -0.2;
        } else {
            lateral = 0;
        }

        // Back the robot up to the right distance to raise the lift
        //axial = -(currentY - targetY) / 40;
        if (aprilTagCurrentY > aprilTagTargetY) {
            axial = -0.15;
        } else {
            axial = 0;
        }

        // Combine the axial, lateral and yaw factors to be powers
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Apply calculated values to drive motors
        lfDrive.setPower(leftFrontPower);
        rfDrive.setPower(rightFrontPower);
        lbDrive.setPower(leftBackPower);
        rbDrive.setPower(rightBackPower);

        // Test to see if we are at all three parts of our desired position and we are aligned.
        if (abs(aprilTagTargetX - aprilTagCurrentX) < 1 && aprilTagCurrentY < aprilTagTargetY && abs(aprilTagTargetAngle - aprilTagCurrentAngle) < 2) {
            aprilTagAligned = true;
            blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        }
        return aprilTagAligned;
    }

    public boolean GoToPose2d(Pose2d targetPosition) {
        double poseTargetX = targetPosition.getX(); // desired X
        double poseTargetY = targetPosition.getY(); // desired Y
        double poseTargetAngle = targetPosition.getRotation().getDegrees(); // desired Angle

        double poseCurrentX = odometry.getPoseMeters().getX();
        double poseCurrentY = odometry.getPoseMeters().getY();
        double poseCurrentAngle = odometry.getPoseMeters().getRotation().getDegrees();

        // Since both the current and target X / Y coords are in field coordinates, so will the delta.
        // Use a PID controller to dampen the error for yaw, fieldX and fieldY.
        double yaw = yawPID.calculate(CalcTurnError(poseTargetAngle, poseCurrentAngle));
        double fieldX = -(poseTargetX - poseCurrentX);;
        double fieldY = poseTargetY - poseCurrentY;;

        // Reset the PID / i gain for distances larger than 0.5m
        if (Math.sqrt((fieldX * fieldX) + (fieldY * fieldY)) > 0.5) {
            axialPID.reset();
            lateralPID.reset();
        }

        pose2dAligned = false;

       // Reorient the field movement requested to robot orientation
        double axial = fieldX * Math.cos(Math.toRadians(poseCurrentAngle)) - fieldY * Math.sin(Math.toRadians(poseCurrentAngle));
        double lateral = fieldX * Math.sin(Math.toRadians(poseCurrentAngle)) + fieldY * Math.cos(Math.toRadians(poseCurrentAngle));

        if (abs (axial) < 0.05) {
            axial = 0;
        }
        if (abs (lateral) < 0.05) {
            lateral = 0;
        }

        axial = axialPID.calculate(axial);
        lateral = lateralPID.calculate(lateral);


        // Combine the axial, lateral and yaw factors to be powers
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
        max = Math.max(max, abs(leftBackPower));
        max = Math.max(max, abs(rightBackPower));

        if (max > 0.7) {
            leftFrontPower  /= (max*1.3);
            rightFrontPower /= (max*1.3);
            leftBackPower   /= (max*1.3);
            rightBackPower  /= (max*1.3);
        }

        // Update Telemetry with key data
        telemetry.addLine(String.format("Pose2D X(Current%5.2f,Target%5.2f)", poseCurrentX, poseTargetX));
        telemetry.addLine(String.format("Pose2D Y(Current%5.1f,Target%5.2f)", poseCurrentY, poseTargetY));
        telemetry.addLine(String.format("Pose2D Φ(Current%5.2f,Target%5.2f)", poseCurrentAngle, poseTargetAngle));
        telemetry.addLine(String.format("Motion (Axial%5.2f,Lateral%5.2f,Yaw%5.2f)", axial, lateral, yaw));
        telemetry.addLine(String.format("Motor Powers(lf:%4.1f,rf:%4.1f,lb%4.1f,rb%4.1f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower));
        telemetry.update();

        // Apply calculated values to drive motors
        lfDrive.setPower(leftFrontPower);
        rfDrive.setPower(rightFrontPower);
        lbDrive.setPower(leftBackPower);
        rbDrive.setPower(rightBackPower);

        // Test to see if we are at all three parts of our desired position and we are aligned.
        if ((abs(poseTargetX - poseCurrentX) < 0.05) && (abs(poseTargetY - poseCurrentY) < 0.05) && abs(poseTargetAngle - poseCurrentAngle) < 3) {
            pose2dAligned = true;
        }
        return pose2dAligned;
    }
}