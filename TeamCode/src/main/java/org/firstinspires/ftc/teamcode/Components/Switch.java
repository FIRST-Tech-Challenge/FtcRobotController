package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


public class Switch {

    DigitalChannel touchSensor;
    public static boolean prezzed = false;
    double lastSwitchTime = -100;
    boolean mode = false;
    public Switch() {
        touchSensor = op.hardwareMap.get(DigitalChannel.class, "clawSwitch");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        mode=false;
    }

        public boolean isSwitched() {
//        touchSensor.
            prezzed = touchSensor.getState()||mode;
        return prezzed;
//            return false;
        }
        public void setMode(boolean p_mode){
        if(time-lastSwitchTime>0.4) {
            lastSwitchTime=time;
            mode = p_mode;
        }
        }
    public boolean getMode(){
        return mode;
    }

    }