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
    public static double armRetract = 0.83;
    public static double armExtend = 0.35;
    public static double armSpecimenExtend = 0;
    public static double wristExtend = 1;
    public static double wristRetract = 0.55;
    public static double wristSpecimenExtend = 0.6;
    public static double armNeutral = 0.75;
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
    public Action servoArm(){
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
    public Action servoArmSpec(){
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
}