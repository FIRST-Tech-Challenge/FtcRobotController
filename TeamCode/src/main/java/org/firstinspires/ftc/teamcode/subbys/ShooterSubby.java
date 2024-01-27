package org.firstinspires.ftc.teamcode.subbys;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

public class ShooterSubby extends SubsystemBase {
    private SimpleServo shooter;
    public ShooterSubby(SimpleServo shoot){
        shooter = shoot;
    }
    public void release(){
        shooter.turnToAngle(90);
    }
    public void bringBack(){
        shooter.turnToAngle(0);
    }
}
