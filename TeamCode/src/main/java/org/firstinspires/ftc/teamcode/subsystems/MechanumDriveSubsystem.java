package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.subsystems.SympleSubSystemBase;

import java.util.HashMap;

public class MechanumDriveSubsystem extends SympleSubSystemBase {
    private final HashMap<MotorNames, MotorEx> motors = new HashMap<>();

    public MechanumDriveSubsystem(RobotController robotController) {
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


    public enum MotorNames {
        FRONT_RIGHT,
        FRONT_LEFT,
        BACK_LEFT,
        BACK_RIGHT
    }
}
