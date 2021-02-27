package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;
import org.firstinspires.ftc.teamcode.toolkit.core.Subsystem;

public class FlickerSubsystem extends Subsystem {

    public UpliftRobot robot;
    private Servo flicker;

    public FlickerSubsystem(UpliftRobot robot){
        super(robot);
        this.robot = robot;
        this.flicker = robot.flicker;
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

    public void flickRing() {
        flicker.setPosition(0.5);
        if(!robot.safeSleep(350)){
            safeDisable();
            return;
        }
        flicker.setPosition(0.15);//0.15
        if(!robot.safeSleep(350)) {
            safeDisable();
            return;
        }
    }
}
