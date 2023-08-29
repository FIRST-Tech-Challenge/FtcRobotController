package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive extends SubsystemBase {
    public Motor leftFront;
    public Motor rightFront;
    public Motor leftBack;
    public Motor rightBack;
    public IMU imu;
    private final Telemetry telemetry;
    private final MecanumDrive mecanumDrive;
    MecanumDriveOdometry mecanumDriveOdometry;
    MecanumDriveKinematics kinematics;
    Pose2d m_pose;
    private ElapsedTime runtime = new ElapsedTime();

    // sets drive speed states
    public final double SPRINT = 0.9;
    public final double NORMAL = 0.6;
    public final double SLOW = 0.4;

    // current state
    public double state = NORMAL;

    /**
     * Initializes drivetrain  and IMU
     * @param hardwareMap hardware map from teleOp for motor setup
     */
    public Drive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftFront = new Motor(hardwareMap, "motorTest1", Motor.GoBILDA.RPM_312);
        rightFront = new Motor(hardwareMap, "motorTest2", Motor.GoBILDA.RPM_312);
        leftBack = new Motor(hardwareMap, "motorTest0", Motor.GoBILDA.RPM_312);
        rightBack = new Motor(hardwareMap, "motorTest3", Motor.GoBILDA.RPM_312);

        // access motors, reverses directions and sets braking behaviour
//        leftFront.motor.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.motor.setDirection(DcMotor.Direction.REVERSE);
//        leftBack.motor.setDirection(DcMotor.Direction.REVERSE);
//        rightBack.motor.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Retrieve the IMU from the hardware map and initialize
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();
        resetEncoders();
        mecanumDrive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);

        Translation2d frontLeftLocation = new Translation2d(0.11, 0.11);
        Translation2d frontRightLocation = new Translation2d(0.11, -0.11);
        Translation2d backLeftLocation = new Translation2d(-0.11, 0.11);
        Translation2d backRightLocation = new Translation2d(-0.11, -0.11);

        kinematics = new MecanumDriveKinematics(
                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        );

        Rotation2d gyroAngle = Rotation2d.fromDegrees(imuAngle());

        mecanumDriveOdometry = new MecanumDriveOdometry(
                kinematics,gyroAngle,
                new Pose2d(0,0, new Rotation2d())
        );
        runtime.reset();
    }

    /**
     * Resets motor encoder velues
     */
    public void resetEncoders() {
        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
//                leftFront.encoder.getRate(), rightFront.encoder.getRate(),
//                leftBack.encoder.getRate(), rightBack.encoder.getRate()
                rightBack.encoder.getRate(), rightBack.encoder.getRate(),
                rightBack.encoder.getRate(), rightBack.encoder.getRate()
        );

        // Get my gyro angle.
        Rotation2d gyroAngle = Rotation2d.fromDegrees(imuAngle());

        // Update the pose
        m_pose = mecanumDriveOdometry.updateWithTime(runtime.seconds(), gyroAngle, wheelSpeeds);

        telemetry.addData("RightRear3 Position", rightBackPos());
        telemetry.addData("Motor Power", rightBackPower());
        telemetry.addData("IMU Value", imuAngle());
        telemetry.addData("Pose x", m_pose.getX());//39.5cm-981/71-1717
        telemetry.addData("Pose y", m_pose.getY());
        telemetry.addData("Run time", runtime);
        telemetry.update();
    }

    public void mecanumFieldDrive(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean squareInputs) {
        mecanumDrive.driveFieldCentric(-strafeSpeed, forwardSpeed, -turnSpeed, imuAngle(), squareInputs);
    }
    public void mecanumCentricDrive(double strafeRightPositive, double forwardPositive, double rotateClockwisePositive, boolean squareInputs) {
        mecanumDrive.driveRobotCentric(-strafeRightPositive, forwardPositive, -rotateClockwisePositive, squareInputs);
    }
    public int rightBackPos() {
        return rightBack.getCurrentPosition();
    }

    public double rightBackPower() {
        return rightBack.get();
    }

    /**
     * Resets IMU heading
     */
    public void resetHeading() {
        imu.resetYaw();
    }

    /**
     * Returns current heading
     * @return Current IMU heading
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    public double imuAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Manually set speed multiplier
     * @param speed Speed multiplier
     */
    public void setSpeed(double speed) {
        state = speed;
    }

    /*
     * Updates robot-centric field movement
     * @param left_stick_y y-position of left stick controlling front-back motion
     * @param left_stick_x x-position of left stick controlling strafing
     * @param right_stick_x x-position of right stick controlling rotation
     */
//    public void updateRobot(double left_stick_y, double left_stick_x, double right_stick_x) {
//        double y = -left_stick_y;
//        double x = left_stick_x * 1.1; // Counteract imperfect strafing
//        double rx = right_stick_x;
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // Keeps all motor powers in proportion
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
//        double frontLeftPower = (y + x + rx) / denominator * state;
//        double backLeftPower = (y - x + rx) / denominator * state;
//        double frontRightPower = (y - x - rx) / denominator * state;
//        double backRightPower = (y + x - rx) / denominator * state;
//
//        // Set powers
//        leftFront.setPower(frontLeftPower);
//        leftBack.setPower(backLeftPower);
//        rightFront.setPower(frontRightPower);
//        rightBack.setPower(backRightPower);
//    }

    /*
     * Updates field-centric robot movement
     * @param left_stick_y y-position of left stick controlling front-back motion
     * @param left_stick_x x-position of left stick controlling strafing
     * @param right_stick_x x-position of right stick controlling rotation
     */
//    public void updateField(double left_stick_y, double left_stick_x, double right_stick_x) {
//        double y = -left_stick_y;
//        double x = left_stick_x * 1.1;
//        double rx = right_stick_x;
//
//        double botHeading = getHeading();
//
//        // Rotate the movement direction counter to the robot's rotation
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
//        double frontLeftPower = (rotY + rotX + rx) / denominator * state;
//        double backLeftPower = (rotY - rotX + rx) / denominator * state;
//        double frontRightPower = (rotY - rotX - rx) / denominator * state;
//        double backRightPower = (rotY + rotX - rx) / denominator * state;
//
//        leftFront.setPower(frontLeftPower);
//        leftBack.setPower(backLeftPower);
//        rightFront.setPower(frontRightPower);
//        rightBack.setPower(backRightPower);
//    }
}
