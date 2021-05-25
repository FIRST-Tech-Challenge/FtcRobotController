package org.firstinspires.ftc.teamcode.robot.parts.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.parts.RobotPart;
import org.firstinspires.ftc.teamcode.robot.parts.RobotPartSettings;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains.HDrive;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains.StandardDrive;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.wheels.WheelTypes;
import org.firstinspires.ftc.teamcode.settings.DeviceNames;

import java.lang.reflect.InvocationTargetException;

/**
 * Settings for Drive Trains
 * @author 22jmiller
 */
public class DriveTrainSettings extends RobotPartSettings {
    public WheelTypes wheelType;
    public boolean useTelemetry;
    public Class driveTrain;
    public boolean leftForwards = false;
    public boolean centerForwards = false;
    public boolean driveWithEncoder = true;

    public DriveTrainSettings(boolean useTelemetry, WheelTypes wheelType, Class driveTrain) {
        if(!(DriveTrain.class.isAssignableFrom(driveTrain))) {
            throw new DriveTrainException(driveTrain + " is not a subclass of DriveTrain");
        }

        this.wheelType = wheelType;
        this.driveTrain = driveTrain;
        this.useTelemetry = useTelemetry;
    }

    /**
     * Will try to auto-detect the drivetrain when create is called, this is not the most reliable test
     * @param useTelemetry
     * @param wheelType
     */
    public DriveTrainSettings(boolean useTelemetry, WheelTypes wheelType) {
        this.wheelType = wheelType;
        this.useTelemetry = useTelemetry;
    }

    public DriveTrain create(Robot robot) {
        if (driveTrain == null) { // Tries to auto-detect
            try {
                robot.hardwareMap.get(DcMotor.class, DeviceNames.CENTER_DRIVE[0]);
                return new HDrive(useTelemetry, wheelType, robot, leftForwards, centerForwards, driveWithEncoder);
            } catch (Exception e) {
                return new StandardDrive(useTelemetry, wheelType, robot);
            }
        } else {
            try {
                return (DriveTrain) driveTrain.getDeclaredConstructor(boolean.class, WheelTypes.class, Robot.class).newInstance(useTelemetry,wheelType,robot);
            } catch (NoSuchMethodException | IllegalAccessException | InstantiationException | InvocationTargetException e) {
                throw new DriveTrainException();
            }
        }
    }
}
