package org.firstinspires.ftc.teamcode.Mechanisms.Extension;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;

@Config
public class Extension {
    HardwareMap hardwareMap;
    Servo servoExtendLeft;
    Servo servoExtendRight;
    public static double extendPos = 0;
    public static double retractPos = 0.27;
    public static double leftOffset = 0.025;
    public Extension(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        servoExtendLeft = hardwareMap.get(Servo.class, "leftExtension");
        servoExtendRight = hardwareMap.get(Servo.class, "rightExtension");
    }

    public enum extensionState {
        RETRACT, //pull extension back
        EXTEND   //push extension forward
    }
    public ElapsedTime timer = new ElapsedTime();
    public Action servoExtension(extensionState extendoPos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                double timeLastUpdate = timer.seconds();
                    if (extendoPos == extensionState.RETRACT) {
                        servoExtendLeft.setPosition(extendPos+leftOffset);
                        servoExtendRight.setPosition(extendPos);
                    } else if (extendoPos == Extension.extensionState.EXTEND) {
                        servoExtendLeft.setPosition(retractPos+leftOffset);
                        servoExtendRight.setPosition(retractPos);
                    }
                return false;
            }
        };
    }
}