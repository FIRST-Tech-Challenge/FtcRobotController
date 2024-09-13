package org.firstinspires.ftc.teamcode.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class DriveSubsystem extends SubsystemBase {
    public MotorEx leftFront, leftRear, rightRear, rightFront;

    public IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public static double TRACK_WIDTH = 12.375;
    public static double CENTER_WHEEL_OFFSET = 0.5; // distance between center of rotation of the robot and the center odometer
    public static double WHEEL_DIAMETER = 1.425;
    public static double TICKS_PER_REV = 8192;
    public static double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    public static boolean BREAK = false;
    public Motor.Encoder leftOdom, rightOdom, centerOdom;
    public HolonomicOdometry odometry;

    public DriveSubsystem(HardwareMap hardwareMap) {
        DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

        imu = hardwareMap.get(IMU.class, "imu");

        leftFront = new MotorEx(hardwareMap, "frontLeft");
        leftFront.setInverted(true);

        leftRear = new MotorEx(hardwareMap, "backLeft");
        leftRear.setInverted(true);

        rightRear = new MotorEx(hardwareMap, "backRight");
        rightRear.setInverted(false);

        rightFront = new MotorEx(hardwareMap, "frontRight");
        rightFront.setInverted(false);

        Motor.ZeroPowerBehavior stop = BREAK ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT;

        rightRear.setZeroPowerBehavior(stop);
        rightFront.setZeroPowerBehavior(stop);
        leftRear.setZeroPowerBehavior(stop);
        leftFront.setZeroPowerBehavior(stop);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        leftOdom = rightRear.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdom = leftFront.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdom = leftRear.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        rightOdom.setDirection(Motor.Direction.REVERSE);

        leftOdom.reset();
        rightOdom.reset();
        centerOdom.reset();

        odometry = new HolonomicOdometry(
                leftOdom::getDistance,
                rightOdom::getDistance,
                centerOdom::getDistance,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        // change to reflect starting field position
        odometry.updatePose(new com.arcrobotics.ftclib.geometry.Pose2d(-60, 60, new Rotation2d(Math.toRadians(0))));
    }

    public void robotCentric(double power, double strafe, double turn) {
        robotCentric(power, strafe, turn, 1, 1);
    }

    public void robotCentric(double power, double strafe, double turn, double multiF, double multiH) {
        double denominator = Math.max(Math.abs(power * 1 / multiF) + Math.abs(strafe * 1 / multiF) + Math.abs(turn * 1 / multiH), Math.max(1, Math.max(1 / multiF, 1 / multiH)));
        double frontLeftPower = (power + strafe + turn) / denominator;
        double backLeftPower = (power - strafe + turn) / denominator;
        double frontRightPower = (power - strafe - turn) / denominator;
        double backRightPower = (power + strafe - turn) / denominator;

        leftFront.set(frontLeftPower);
        leftRear.set(backLeftPower);
        rightFront.set(frontRightPower);
        rightRear.set(backRightPower);
    }

    public void updatePos() {
        odometry.updatePose();
    }

    public Pose2d getPos() {
        return odometry.getPose();
    }
}
