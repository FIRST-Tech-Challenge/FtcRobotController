package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.UpliftRobot;

public class FlickerFunctions {

    UpliftRobot robot;
    Servo flicker;
    AnalogInput potentiometer;

    public FlickerFunctions(UpliftRobot robot){
        this.robot = robot;
        this.flicker = robot.flicker;
        this.potentiometer = robot.potentiometer;
    }

    public void setFlickerPos(double pos) {
        flicker.setPosition(pos);
    }

    public void setFlickerOut() {
        flicker.setPosition(0.49);
    }

    public void setFlickerIn() {
        flicker.setPosition(0.618);
    }

    public void flickRing() {
        // move in
        setFlickerIn();
        double initialTime = System.currentTimeMillis();
        while(potentiometer.getVoltage() > 0.635 && System.currentTimeMillis() - initialTime < 750) {
            robot.safeSleep(1);
        }
        setFlickerOut();
        initialTime = System.currentTimeMillis();
        while(potentiometer.getVoltage() < 0.99 && System.currentTimeMillis() - initialTime < 750) {
            robot.safeSleep(1);
        }
    }

}
