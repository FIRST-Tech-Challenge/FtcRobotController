package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robot.op;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

public class LimitSwitches {
    private RevTouchSensor limitSwitchRight, limitSwitchLeft;
    private double[] pos = {0,0,0};
    private double robotWidth = 15;
    public LimitSwitches() {
//            limitSwitchLeft = opMode.hardwareMap.get(DigitalChannel.class, "limitSwitchLeft");
            limitSwitchRight = op.hardwareMap.get(RevTouchSensor.class, "limitSwitchRight");
            limitSwitchRight.resetDeviceConfigurationForOpMode();
//            limitSwitchLeft.setMode(DigitalChannel.Mode.INPUT);
//            limitSwitchRight.setMode(DigitalChannel.Mode.INPUT);
    }
    public boolean updateTouch(double xpos, double ypos, double angle) {
        pos[0]=xpos;
        pos[1]=ypos;
        pos[2]=angle;
        angle *= 180 / PI;
        if (limitSwitchRight.isPressed()) {
            if (abs(angle) < 5 || abs(360-angle)<5) {
                pos[1]= -70.5 + robotWidth/2;
                return true;
            } else if (abs(90 - angle) < 5) {
                pos[0] = 70.5 - robotWidth/2;
                return true;
            } else if (abs(angle) > 175 && abs(angle) < 185) {
                pos[1] = 70.5- robotWidth/2;
                return true;
            } else if (abs(-90 - angle) < 5) {
                pos[0] = -70.5 + robotWidth/2;
                return true;
            }
        }
//        if (limitSwitchLeft.getState()) {
//            if (abs(angle) < 5) {
//                pos[0] = -70.5+ robotWidth/2;
//                return true;
//            } else if (abs(90 - angle) < 5) {
//                pos[1] = 70.5 - robotWidth/2;
//                return true;
//            } else if (abs(angle) > 175) {
//                pos[0] = 70.5 - robotWidth/2;
//                return true;
//            } else if (abs(-90 - angle) < 5) {
//                pos[1] = -70.5+ robotWidth/2;
//                return true;
//            }
//        }
        return false;
    }
    public Pose2d getPose2d(){
        return new Pose2d(pos[0],pos[1],pos[2]);
    }

}
