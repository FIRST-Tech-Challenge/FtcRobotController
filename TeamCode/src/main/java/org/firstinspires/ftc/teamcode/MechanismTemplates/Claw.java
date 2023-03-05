package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SignalEdgeDetector;

import java.util.function.BooleanSupplier;

@Config
public class Claw {
    public Servo wristJoint;
    public Servo clawJoint; // public because a button must set the wrist to the intake position regardless of its status

    // claw positions
    public static double OPEN = 0.7;
    public static double CLOSE = 0.5;

    // wrist positions
    public static double WRIST_INTAKE_POSITION = 0.255; // wrist rotates to intake cone, greater values move clockwise, less move counterclockwise
    public static double WRIST_EXTAKE_POSITION = 0.915; // wrist rotates to extake on junction

    public Claw(HardwareMap hardwareMap) {
        // Control Hub Pins
        wristJoint = hardwareMap.get(Servo.class, "WRIST"); // Pin 0
        clawJoint = hardwareMap.get(Servo.class, "CLAW"); // Pin 1

        // Presets for teleOp
        clawJoint.setPosition(CLOSE);
        wristJoint.setPosition(WRIST_INTAKE_POSITION);
    }

    public void toggleOpenClose(){
        if(clawJoint.getPosition() == OPEN){
            clawJoint.setPosition(CLOSE);
        }else {
            clawJoint.setPosition((OPEN));
        }
    }

    public void toggleWristRotate(){
        if (wristJoint.getPosition() == WRIST_EXTAKE_POSITION) {
            wristJoint.setPosition(WRIST_INTAKE_POSITION);
        } else {
            wristJoint.setPosition(WRIST_EXTAKE_POSITION);
        }
    }
}
