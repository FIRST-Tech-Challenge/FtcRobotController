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
    private boolean isAuto;

    // claw positions
    private final double OPEN = 0.69;
    private final double CLOSE = 0.44;

    // wrist positions
    public final double WRIST_INTAKE_POSITION = 0.72; // wrist rotates to intake cone, greater values move clockwise, less move counterclockwise
    private final double WRIST_EXTAKE_POSITION = 0.05; // wrist rotates to extake on junction

    SignalEdgeDetector isOpen;
    SignalEdgeDetector isIntakePosition;

    public Claw(HardwareMap hardwareMap, boolean isAuton, BooleanSupplier rightBumper, BooleanSupplier aButton) {
        isAuto = isAuton;

        // Control Hub Pins
        wristJoint = hardwareMap.get(Servo.class, "WRIST"); // Pin 0
        clawJoint = hardwareMap.get(Servo.class, "CLAW"); // Pin 1

        // Presets for teleOp
        clawJoint.setPosition(OPEN);
        wristJoint.setPosition(WRIST_INTAKE_POSITION);

        isOpen = new SignalEdgeDetector(rightBumper);
        isIntakePosition = new SignalEdgeDetector(aButton);
    }

    // Overloaded method for autonomous
    public Claw(HardwareMap hardwareMap){
        clawJoint.setPosition(CLOSE);
        wristJoint.setPosition(WRIST_INTAKE_POSITION);
    }

    private boolean clawToggled;

    public void toggleOpenClose() {
        if (isOpen.isRisingEdge()) {
            if (clawToggled) {
                clawJoint.setPosition(CLOSE);
            } else {
                clawJoint.setPosition(OPEN);
            }
            clawToggled = !clawToggled;
        }
    }

    public boolean wristInExtakePosition;

    public void toggleWristRotate() { // would be used at the beginning of the goToJunction method in the arm class, for example
        if (wristInExtakePosition) {
            wristJoint.setPosition(WRIST_INTAKE_POSITION);
        } else { // if the wrist is in the extake position, switch it back to the intake position so it can pick up cones
            wristJoint.setPosition(WRIST_EXTAKE_POSITION);
        }
        wristInExtakePosition = !wristInExtakePosition;
    }

    public void update() {
        isOpen.update();
        isIntakePosition.update();
    }
}
