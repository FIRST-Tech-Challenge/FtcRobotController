package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.subsystems.bases.DriveTrainBaseSubsystem;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.subsystems.SympleSubsystemBase;

import java.util.HashMap;

public class MecanumDriveSubsystem extends SympleSubsystemBase implements DriveTrainBaseSubsystem {
    private final HashMap<MotorNames, MotorEx> motors = new HashMap<>();

    public MecanumDriveSubsystem(RobotController robotController) {
        super(robotController);
        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Getting motors");
        this.motors.put(MotorNames.FRONT_RIGHT, new MotorEx(robotController.getHardwareMap(), MotorMap.FRONT_RIGHT.getId()));
        this.motors.put(MotorNames.FRONT_LEFT, new MotorEx(robotController.getHardwareMap(), MotorMap.FRONT_LEFT.getId()));
        this.motors.put(MotorNames.BACK_LEFT, new MotorEx(robotController.getHardwareMap(), MotorMap.BACK_LEFT.getId()));
        this.motors.put(MotorNames.BACK_RIGHT, new MotorEx(robotController.getHardwareMap(), MotorMap.BACK_RIGHT.getId()));

        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Inverting motors");
        this.motors.get(MotorNames.FRONT_RIGHT).setInverted(true);
        this.motors.get(MotorNames.BACK_RIGHT).setInverted(true);
    }

    public void moveMotor(MotorNames motor, double power) {
        MotorEx m = this.motors.get(motor);
        if (m == null) {
            this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": failed to set power to the motor '" + motor.name() + "'");
            return;
        }
        m.set(power);
    }

    @Override
    public void moveSideMotors(double left, double right) {
        this.moveMotor(MotorNames.FRONT_LEFT, left);
        this.moveMotor(MotorNames.BACK_LEFT, left);

        this.moveMotor(MotorNames.FRONT_RIGHT, right);
        this.moveMotor(MotorNames.BACK_RIGHT, right);
    }

    @Override
    public double getForwardDistanceDriven() {
        return this.robotController.getRobotPositionManager().getRightWheelDistanceDriven();
    }

    @Override
    public double getHeading() {
        return this.robotController.getRobotPositionManager().getHeadingByGyro();
    }

    public enum MotorNames {
        FRONT_RIGHT,
        FRONT_LEFT,
        BACK_LEFT,
        BACK_RIGHT
    }
}
