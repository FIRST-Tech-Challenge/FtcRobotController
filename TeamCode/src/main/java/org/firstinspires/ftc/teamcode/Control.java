package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Control extends OpMode {
    Hraezlyr hraezlyr;
    enum Level {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }


    @Override
    public void init() {
        hraezlyr = new Hraezlyr(hardwareMap);
    }

    public void cascadeLift(Level zHeight){
        switch(zHeight){
            case GROUND:
                hraezlyr.cascadeMotor1.setTargetPosition(0);
                hraezlyr.cascadeMotor2.setTargetPosition(0);
                // all target positions are not accurate to the motor
            case LOW:
                hraezlyr.cascadeMotor1.setTargetPosition(1000);
                hraezlyr.cascadeMotor2.setTargetPosition(1000);
            case MEDIUM:
                hraezlyr.cascadeMotor1.setTargetPosition(2000);
                hraezlyr.cascadeMotor2.setTargetPosition(2000);
            case HIGH:
                hraezlyr.cascadeMotor1.setTargetPosition(3000);
                hraezlyr.cascadeMotor2.setTargetPosition(3000);
        }
    }
}