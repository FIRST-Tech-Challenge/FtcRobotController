package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;
import org.firstinspires.ftc.teamcode.toolkit.core.Subsystem;

public class FlickerSubsystem extends Subsystem {

    public UpliftRobot robot;
    private Servo flicker;
    public AnalogInput potentiometer;

    public FlickerSubsystem(UpliftRobot robot){
        super(robot);
        this.robot = robot;
        this.flicker = robot.flicker;
        this.potentiometer = robot.potentiometer;
    }
    @Override
    public void enable() {
        
    }

    @Override
    public void disable() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void safeDisable() {

    }

    public void setFlickerPos(double pos) {
        flicker.setPosition(pos);
    }

//    public void flickRing() {
//        robot.setFlickingState(UpliftRobot.FlickingState.FLICKING);
//        // move in
//        flicker.setPosition(0.22);
//        robot.safeSleep(350);
//
//        // move out
//        flicker.setPosition(0);
//        robot.safeSleep(350);
//        robot.setFlickingState(UpliftRobot.FlickingState.IDLE);
//    }

    public void setFlickerOut() {
        flicker.setPosition(0.47);
    }

    public void setFlickerIn() {
        flicker.setPosition(0.59);
    }

    public void flickRing() {
        robot.setFlickingState(UpliftRobot.FlickingState.FLICKING);
        // move in
        setFlickerIn();
        double initialTime = System.currentTimeMillis();
        while(potentiometer.getVoltage() > 0.575 && System.currentTimeMillis() - initialTime < 750) {
            robot.safeSleep(1);
            Log.i("Flicker", potentiometer.getVoltage() + "");
        }
        Log.i("Flicker", "In Time: " + (System.currentTimeMillis() - initialTime));
        setFlickerOut();
        initialTime = System.currentTimeMillis();
        while(potentiometer.getVoltage() < 0.88 && System.currentTimeMillis() - initialTime < 750) {
            robot.safeSleep(1);
            Log.i("Flicker", potentiometer.getVoltage() + "");
        }
        Log.i("Flicker", "Out Time: " + (System.currentTimeMillis() - initialTime));
    }
}
