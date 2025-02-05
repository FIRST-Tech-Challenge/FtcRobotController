package org.firstinspires.ftc.teamcode.opmodes.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawYaw {
    private Servo clawYawServo;

    private static final double YAW_LEFT = 0.0;   // Face left
    private static final double YAW_RIGHT = 0.5;  // Face right
    private static final double YAW_CENTER = 0.5; // Neutral (forward)

    public ClawYaw(Servo clawYawServo) {
        this.clawYawServo = clawYawServo;
        resetYaw();
    }

    public void pointLeft() {
        clawYawServo.setPosition(YAW_LEFT);
    }

    public void pointRight() {
        clawYawServo.setPosition(YAW_RIGHT);
    }

    public void resetYaw() {
        clawYawServo.setPosition(0.5);
    }
}
