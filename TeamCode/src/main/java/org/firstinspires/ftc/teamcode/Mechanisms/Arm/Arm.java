package org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware.Actuators.ServoAdvanced;

@Config
public class Arm {
    HardwareMap hardwareMap;
    ServoAdvanced servoWrist;
    public ServoAdvanced servoArmLeft;
    ServoAdvanced servoArmRight;
    public static double armRetract = 0.85;
    public static double armExtend = 0.3;
    public static double armSpecimenExtend = 0;
    public static double wristExtend = 1;
    public static double wristRetract = 0.5;
    public static double wristSpecimenExtend = 0.66;
    public static double armNeutral = 0.65;
    public static double armAuton = 0.5;
    public static double wristAuton = 0.5;
    public ElapsedTime timer = new ElapsedTime();
    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.servoWrist = new ServoAdvanced(hardwareMap.get(Servo.class, "wrist"));
        this.servoArmLeft = new ServoAdvanced(hardwareMap.get(Servo.class, "armRight"));
        this.servoArmRight = new ServoAdvanced(hardwareMap.get(Servo.class, "armLeft"));
    }

    public enum armState {
        RETRACT,    // pulls arm in
        EXTEND,      // pushes arm out
        SPEC
    }
    armState armPos = armState.RETRACT;
    ElapsedTime armTimer = new ElapsedTime();
    public Action armSample(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                    double time = armTimer.seconds();
                    if (time > .3) {
                        if (armPos == armState.RETRACT) {
                            servoArmLeft.setPosition(armExtend);
                            servoArmRight.setPosition(armExtend);
                            servoWrist.setPosition(wristExtend);
                            armPos = armState.EXTEND;
                            armTimer.reset();
                        } else if (armPos == armState.EXTEND) {
                            servoArmLeft.setPosition(armRetract);
                            servoArmRight.setPosition(armRetract);
                            servoWrist.setPosition(wristRetract);
                            armPos = armState.RETRACT;
                            armTimer.reset();
                        }
                    }
                return false;
            }
        };
    }
    public Action armRetract(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {

                        servoArmLeft.setPosition(armRetract);
                        servoArmRight.setPosition(armRetract);
                        servoWrist.setPosition(wristRetract);
                return false;
            }
        };
    }
    public Action armExtend(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                servoArmLeft.setPosition(armExtend);
                servoArmRight.setPosition(armExtend);
                servoWrist.setPosition(wristExtend);
                return false;
            }
        };
    }
    public Action armNeutral(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet){
                servoArmLeft.setPosition(armNeutral);
                servoArmRight.setPosition(armNeutral);
                servoWrist.setPosition(wristRetract);
                return false;
            }
        };
    }
    public Action armSpecimen(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                double time = armTimer.seconds();
                if (time > .3) {
                    if (armPos == armState.RETRACT) {
                        servoArmLeft.setPosition(armSpecimenExtend);
                        servoArmRight.setPosition(armSpecimenExtend);
                        servoWrist.setPosition(wristSpecimenExtend);
                        armPos = armState.SPEC;
                        armTimer.reset();
                    } else if (armPos == armState.SPEC) {
                        servoArmLeft.setPosition(armRetract);
                        servoArmRight.setPosition(armRetract);
                        servoWrist.setPosition(wristRetract);
                        armPos = armState.RETRACT;
                        armTimer.reset();
                    }
                }
                return false;
            }
        };
    }
    public Action specimenAuton(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                servoArmLeft.setPosition(armAuton);
                servoArmRight.setPosition(armAuton);
                servoWrist.setPosition(wristAuton);
                return false;
            }
        };
    }
}