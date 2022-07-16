package org.firstinspires.ftc.teamcode.Components;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

public class LimitSwitches {
    private LinearOpMode op = null;
    private DigitalChannel limitSwitchRight, limitSwitchLeft;
    private double[] pos = {0,0,0};
    public LimitSwitches(LinearOpMode opMode) {
            limitSwitchLeft = opMode.hardwareMap.get(DigitalChannel.class, "limitSwitchLeft");
            limitSwitchRight = opMode.hardwareMap.get(DigitalChannel.class, "limitSwitchRight");
            limitSwitchLeft.setMode(DigitalChannel.Mode.INPUT);
            limitSwitchRight.setMode(DigitalChannel.Mode.INPUT);
    }
    public boolean updateTouch(double xpos, double ypos, double angle) {
        pos[0]=xpos;
        pos[1]=ypos;
        pos[2]=angle;
        if (limitSwitchRight.getState()) {
            if (abs(angle) < 5) {
                pos[0]= 70.5;
                return true;
            } else if (abs(90 - angle) < 5) {
                pos[1] = -70.5;
                return true;
            } else if (abs(angle) > 175) {
                pos[0] = -70.5;
                return true;
            } else if (abs(-90 - angle) < 5) {
                pos[1] = 70.5;
                return true;
            }
        }
        if (limitSwitchLeft.getState()) {
            if (abs(angle) < 5) {
                pos[0] = -70.5;
                return true;
            } else if (abs(90 - angle) < 5) {
                pos[1] = 70.5;
                return true;
            } else if (abs(angle) > 175) {
                pos[0] = 70.5;
                return true;
            } else if (abs(-90 - angle) < 5) {
                pos[1] = -70.5;
                return true;
            }
        }
        return false;
    }
    public Pose2d getPose2d(){
        return new Pose2d(pos[0],pos[1],pos[2]);
    }

}
