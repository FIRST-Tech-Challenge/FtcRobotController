package org.firstinspires.ftc.teamcode.opmodes.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;

public class ClawCode {
    private Servo clawOpenCloseServo;
    private ClawPitch clawPitch;
    private ClawYaw clawYaw;
    private ClawRoll clawRoll;

    private static final double CLAW_OPEN_POSITION = 0.8;   // Fully open
    private static final double CLAW_CLOSED_POSITION = 0.5; // Fully closed
    private static final double CLAW_NEUTRAL_POSITION = 0.5; // Neutral (half-open)

    private boolean isClawOpen = false;  // Track claw state (open/closed/neutral)
    private boolean isRollNormal = true; // Track roll state (normal/reverse)
    private boolean isYawLeft = false;  // Track yaw state (left/right)
    private boolean isPitchPickup = false;  // Track pitch state (pickup/neutral)

    public ClawCode(Servo clawOpenCloseServo, Servo clawRotationServo, Servo clawYawServo, Servo clawPitchServo) {
        this.clawOpenCloseServo = clawOpenCloseServo;
        this.clawPitch = new ClawPitch(clawPitchServo);
        this.clawYaw = new ClawYaw(clawYawServo);
        this.clawRoll = new ClawRoll(clawRotationServo);
        clawRoll.rotateNormal(); // Set initial position for roll

        resetClaw(); // Set claw to default neutral state
    }

    private boolean prevYPressed = false;  // Track previous button state

    public void controlClawLogitech(Gamepad logitechGamepad) {
        boolean currentYPressed = logitechGamepad.y;

        // Toggle claw only when Y is pressed and was not pressed in the previous cycle
        if (currentYPressed && !prevYPressed) {
            if (!isClawOpen) {
                clawOpenCloseServo.setPosition(CLAW_OPEN_POSITION);  // Open claw
            } else {
                clawOpenCloseServo.setPosition(CLAW_CLOSED_POSITION);  // Close claw
            }
            isClawOpen = !isClawOpen;  // Toggle state
        }

        prevYPressed = currentYPressed;  // Update previous state


    }


    private boolean prevCirclePressed = false;  // Track previous button state
    private boolean prevBumperPressed = false;
    public void controlClawPS4(Gamepad ps4Gamepad) {
        boolean currentCirclePressed = ps4Gamepad.circle;
        boolean currentBumperPressed = ps4Gamepad.right_bumper;

        // Toggle claw only when Circle is pressed and was not pressed in the previous cycle
        if (currentCirclePressed && !prevCirclePressed) {
            if (!isPitchPickup) {
                clawPitch.setNeutralPosition();
            } else {
                clawPitch.setPickupPosition();
            }
            isPitchPickup = !isPitchPickup;  // Toggle state
        }

        prevCirclePressed = currentCirclePressed;  // Update previous state

    if(currentBumperPressed && !prevBumperPressed){
        if(!isRollNormal){
            clawRoll.resetRoll();  // Reverse claw roll

        }else{
            clawRoll.rotateNormal();  // Normal claw roll
        }
        isRollNormal = !isRollNormal;
    }
        prevBumperPressed = currentBumperPressed;
        // Reset all positions using the "Share" button


    }


    public void submerisibleClaw() {
        clawOpenCloseServo.setPosition(CLAW_CLOSED_POSITION);  // Set claw to neutral
        clawPitch.setHangPosition();  // Reset pitch to neutral
        clawYaw.resetYaw();  // Reset yaw to neutral position
        clawRoll.resetRoll();  // Reset roll to neutral position
        isClawOpen = false;  // Reset claw state to neutral
        isRollNormal = true;  // Reset roll state to normal
        isYawLeft = false;  // Reset yaw state to neutral
        isPitchPickup = false;  // Reset pitch state to neutral
    }



    public void resetClaw() {
        clawOpenCloseServo.setPosition(CLAW_OPEN_POSITION);  // Set claw to neutral
        clawPitch.setNeutralPosition();  // Reset pitch to neutral
        clawYaw.resetYaw();  // Reset yaw to neutral position
        clawRoll.rotateNormal();  // Reset roll to neutral position
        isClawOpen = false;  // Reset claw state to neutral
        isRollNormal = true;  // Reset roll state to normal
        isYawLeft = false;  // Reset yaw state to neutral
        isPitchPickup = false;  // Reset pitch state to neutral
    }

    private boolean isClawNeutral() {
        return clawOpenCloseServo.getPosition() == CLAW_NEUTRAL_POSITION;
    }
}


//package org.firstinspires.ftc.teamcode.opmodes.Subsystems;
//
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
//
//public class ClawCode {
//    private Servo clawOpenCloseServo;
//    private ClawPitch clawPitch;
//    private ClawYaw clawYaw;
//    private ClawRoll clawRoll;
//
//    private static final double CLAW_OPEN_POSITION = 0.8;   // Fully open
//    private static final double CLAW_CLOSED_POSITION = 0.5; // Fully closed
//    private static final double CLAW_NEUTRAL_POSITION = 0.5; // Neutral (half-open)
//
//    public ClawCode(Servo clawOpenCloseServo, Servo clawRotationServo, Servo clawYawServo, Servo clawPitchServo) {
//        this.clawOpenCloseServo = clawOpenCloseServo;
//        this.clawPitch = new ClawPitch(clawPitchServo);
//        this.clawYaw = new ClawYaw(clawYawServo);
//        this.clawRoll = new ClawRoll(clawRotationServo);
//        clawRoll.rotateNormal();
//
//        resetClaw();
//    }
//
//    public void controlClaw(Gamepad gamepad) {
//
//
//        if (gamepad.left_bumper) {
//            clawOpenCloseServo.setPosition(CLAW_OPEN_POSITION);
//            clawRoll.rotateNormal();
//        } else if (gamepad.right_bumper) {
//            clawOpenCloseServo.setPosition(CLAW_CLOSED_POSITION);
//            clawRoll.rotateNormal();
//        } else if (gamepad.share) {
//            clawOpenCloseServo.setPosition(CLAW_NEUTRAL_POSITION);
//        }
//
//        if (gamepad.triangle) {
//            clawPitch.setPickupPosition();
//        } else if (gamepad.cross) {
//            clawPitch.setNeutralPosition();
//        } else if (gamepad.share) {
//            clawPitch.setNeutralPosition();
//        }
//
//        if (gamepad.square) {
//            clawYaw.pointLeft();
//        } else if (gamepad.circle) {
//            clawYaw.pointRight();
//        } else if (gamepad.share) {
//            clawYaw.resetYaw();
//        }
//
//
//        // Additional controls for pitch can be added as needed.
//    }
//
//    public void resetClaw() {
//        clawOpenCloseServo.setPosition(CLAW_NEUTRAL_POSITION);
//        clawPitch.resetPitch();
//        clawYaw.resetYaw();
//        clawRoll.resetRoll();
//    }
//}
