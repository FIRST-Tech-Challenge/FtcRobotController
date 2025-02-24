package org.firstinspires.ftc.teamcode.subsystems.driveTrain;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.managers.RobotPositionManager;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.drivetrain.MecanumChassisUtils;
import org.firstinspires.ftc.teamcode.util.drivetrain.MecanumWheelSet;

import java.util.HashMap;

public class MecanumDriveSubsystem extends SubsystemBase implements IDriveTrainSubsystem {
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;
    private final MecanumWheelSet wheelSet;

    private DriveConstants.DriveSpeed driveSpeedModifier = DriveConstants.DriveSpeed.NORMAL;

    public MecanumDriveSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Getting motors");
        this.wheelSet = new MecanumWheelSet(
                new MotorEx(hardwareMap, MotorMap.LEG_FRONT_LEFT.getId()), new MotorEx(hardwareMap, MotorMap.LEG_FRONT_RIGHT.getId()),
                new MotorEx(hardwareMap, MotorMap.LEG_BACK_LEFT.getId()), new MotorEx(hardwareMap, MotorMap.LEG_BACK_RIGHT.getId())
        );

        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Inverting motors");
        this.wheelSet.setInverted(MecanumWheelSet.MecanumWheel.FRONT_RIGHT, true);
        this.wheelSet.setInverted(MecanumWheelSet.MecanumWheel.BACK_RIGHT, true);

        this.wheelSet.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        this.getTelemetry().addData("Robot Angle", this.getHeading());
    }

    public void moveMotor(MecanumWheelSet.MecanumWheel wheel, double power) {
        this.wheelSet.setPower(wheel, power);
    }
    
    public void setAllChassisPower(double power) {
        this.wheelSet.setPower(power);
    }

    @Override
    public void moveSideMotors(double left, double right) {
        this.wheelSet.setSidePower(left, right);
    }

    public void moveMotors(MecanumChassisUtils.MecanumWheelSpeeds mecanumWheelSpeeds) {
        this.wheelSet.setPower(mecanumWheelSpeeds);
    }

    public DriveConstants.DriveSpeed getDriveSpeedModifier() {
        return driveSpeedModifier;
    }

    public void setDriveSpeedModifier(DriveConstants.DriveSpeed driveSpeedModifier) {
        this.driveSpeedModifier = driveSpeedModifier;
    }

    @Override
    public double getForwardDistanceDriven() {
        return RobotPositionManager.getInstance().getRightWheelDistanceDriven();
    }

    public double getSideDistanceDriven() {
        return RobotPositionManager.getInstance().getBackWheelDistanceDriven();
    }

    @Override
    public double getHeading() {
        return RobotPositionManager.getInstance().getRelativeHeading();
    }

    @Override
    public DataLogger getDataLogger() {
        return this.dataLogger;
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return this.telemetry;
    }

    public enum MotorNames {
        FRONT_RIGHT,
        FRONT_LEFT,
        BACK_LEFT,
        BACK_RIGHT
    }
}
