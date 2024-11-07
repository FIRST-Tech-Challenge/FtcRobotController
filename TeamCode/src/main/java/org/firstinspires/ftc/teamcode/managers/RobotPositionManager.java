package org.firstinspires.ftc.teamcode.managers;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maps.SensorMap;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.DriveConstants;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class RobotPositionManager {
    private final BHI260IMU imu;
    private final MotorEx rightDeadWheel;
    private final MotorEx leftDeadWheel;
    private final MotorEx backDeadWheel;

    private final double startingAngle;

    private static RobotPositionManager instance;

    private RobotPositionManager(HardwareMap hardwareMap) {
        BHI260IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(DriveConstants.LOGO_FACING_DIRECTION, DriveConstants.USB_FACING_DIRECTION));
        this.imu = hardwareMap.get(BHI260IMU.class, "imu");
        this.imu.initialize(parameters);

        this.rightDeadWheel = new MotorEx(hardwareMap, SensorMap.DEAD_WHEEL_RIGHT.getId());
        this.leftDeadWheel = new MotorEx(hardwareMap, SensorMap.DEAD_WHEEL_LEFT.getId());
        this.backDeadWheel = new MotorEx(hardwareMap, SensorMap.DEAD_WHEEL_BACK.getId());

        this.rightDeadWheel.encoder.setDirection(Motor.Direction.REVERSE);
        this.leftDeadWheel.encoder.setDirection(Motor.Direction.REVERSE);
//        this.rightDeadWheel.encoder.setDirection(Motor.Direction.REVERSE);

        this.rightDeadWheel.resetEncoder();
        this.leftDeadWheel.resetEncoder();
        this.backDeadWheel.resetEncoder();

        this.startingAngle = getHeadingByGyro();
    }

    public static void init(HardwareMap hardwareMap) {
        instance = new RobotPositionManager(hardwareMap);
    }

    public static RobotPositionManager getInstance() {
        return instance;
    }

    public double getHeadingByGyro() {
        return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getRelativeHeading() {
        return this.getHeadingByGyro() - this.startingAngle;
    }

    public double getHeadingByWheels() {
        double right = this.getRightWheelDistanceDriven();
        double left = this.getLeftWheelDistanceDriven();
        return Math.toDegrees((right - left) / DriveConstants.WHEELS_DISTANCE);
    }

    public double getLeftWheelDistanceDriven() {
        return MathUtil.encoderTicksToMeter(this.leftDeadWheel.getCurrentPosition());
    }

    public double getRightWheelDistanceDriven() {
        return MathUtil.encoderTicksToMeter(this.rightDeadWheel.getCurrentPosition());
    }

    public double getBackWheelDistanceDriven() {
        return MathUtil.encoderTicksToMeter(this.backDeadWheel.getCurrentPosition());
    }
}
