package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Control extends OpMode {
    Haezler haezler;
    enum Level {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }


    @Override
    public void init() {
        haezler = new Haezler(hardwareMap);
    }

    public void cascadeLift(Level zHeight){
        switch(zHeight){
            case GROUND:
                haezler.cascadeMotor1.setTargetPosition(0);
                haezler.cascadeMotor2.setTargetPosition(0);
                // all target positions are not accurate to the motor
            case LOW:
                haezler.cascadeMotor1.setTargetPosition(1000);
                haezler.cascadeMotor2.setTargetPosition(1000);
            case MEDIUM:
                haezler.cascadeMotor1.setTargetPosition(2000);
                haezler.cascadeMotor2.setTargetPosition(2000);
            case HIGH:
                haezler.cascadeMotor1.setTargetPosition(3000);
                haezler.cascadeMotor2.setTargetPosition(3000);
        }
    }
}
