package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.maps.MotorMap;

import java.util.HashMap;

public class MechanumDriveSubsystem extends SubsystemBase {
    private final HashMap<MotorNames, MotorEx> motors = new HashMap<>();

    public MechanumDriveSubsystem(RobotController robotController){
        this.motors.put(MotorNames.FRONT_RIGHT, new MotorEx(robotController.getHardwareMap(), MotorMap.FRONT_RIGHT.getId()));
        this.motors.put(MotorNames.FRONT_LEFT, new MotorEx(robotController.getHardwareMap(), MotorMap.FRONT_LEFT.getId()));
        this.motors.put(MotorNames.BACK_LEFT, new MotorEx(robotController.getHardwareMap(), MotorMap.BACK_LEFT.getId()));
        this.motors.put(MotorNames.BACK_RIGHT, new MotorEx(robotController.getHardwareMap(), MotorMap.BACK_RIGHT.getId()));

        this.motors.get(MotorNames.FRONT_RIGHT).setInverted(true);
        this.motors.get(MotorNames.BACK_RIGHT).setInverted(true);
    }
    public void moveMotor(MotorNames motor,double power){
        MotorEx m = this.motors.get(motor);
        if(m == null) return;
        m.set(power);
    }


    public enum MotorNames {
        FRONT_RIGHT,
        FRONT_LEFT,
        BACK_LEFT,
        BACK_RIGHT
    }
}
