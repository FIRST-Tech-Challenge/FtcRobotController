package org.firstinspires.ftc.teamcode.common.subsystems;
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

public class DriveSubsystem extends SubsystemBase {
    public MotorEx leftFront, leftRear, rightRear, rightFront;

    private IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public static final double TRACKWIDTH = 12;
    public static final double CENTER_WHEEL_OFFSET = 0; // distance between center of rotation of the robot and the center odometer
    public static final double WHEEL_DIAMETER = 2.0;
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    public Motor.Encoder leftOdom, rightOdom, centerOdom;
    public HolonomicOdometry odometry;

    public DriveSubsystem(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

        leftFront = new MotorEx(hardwareMap, "frontLeft");
        leftFront.setInverted(true);

        leftRear = new MotorEx(hardwareMap, "backLeft");
        leftRear.setInverted(true);

        rightRear = new MotorEx(hardwareMap, "backRight");
        rightRear.setInverted(false);

        rightFront = new MotorEx(hardwareMap, "frontRight");
        rightFront.setInverted(false);

        rightRear.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        leftOdom = rightFront.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdom = rightRear.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdom = leftRear.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        rightOdom.setDirection(Motor.Direction.REVERSE);

        leftOdom.reset();
        rightOdom.reset();
        centerOdom.reset();

        odometry = new HolonomicOdometry(
                leftOdom::getDistance,
                rightOdom::getDistance,
                centerOdom::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        // change to reflect starting field position
        odometry.updatePose(new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d(0)));
    }

    public void robotCentric(double power, double strafe, double turn) {
        double denominator = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
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
