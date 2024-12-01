package org.firstinspires.ftc.teamcode.Mechanisms.Arm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Arm {
    HardwareMap hardwareMap;
    Servo servoWrist;
    public Servo servoArmLeft;
    Servo servoArmRight;
    public static double armRetract = 1;
    public static double armExtend = -1;
    public static double wristExtend = 1;
    public static double wristRetract = 0.5;
    public ElapsedTime timer = new ElapsedTime();
    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.servoWrist = hardwareMap.get(Servo.class, "wrist");
        this.servoArmLeft = hardwareMap.get(Servo.class, "armRight");
        this.servoArmRight = hardwareMap.get(Servo.class, "armLeft");
    }

    public enum armState {
        RETRACT,    // pulls arm in
        EXTEND      // pushes arm out
    }
    public armState armPos = armState.RETRACT;
    public Action servoArm(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                double timeLastUpdate = timer.seconds();
                if (timeLastUpdate > 0.5) {
                    if (armPos == armState.RETRACT) {
                        servoArmLeft.setPosition(armExtend);
                        servoArmRight.setPosition(armExtend);
                        servoWrist.setPosition(wristExtend);
                        armPos = armState.EXTEND;
                    } else if (armPos == armState.EXTEND) {
                        servoArmLeft.setPosition(armRetract);
                        servoArmRight.setPosition(armRetract);
                        servoWrist.setPosition(wristRetract);
                        armPos = armState.RETRACT;
                    }
                    timer.reset();
                }
                return false;
            }
        };
    }
}