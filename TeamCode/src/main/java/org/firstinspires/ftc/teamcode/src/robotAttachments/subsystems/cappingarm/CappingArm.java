package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.cappingarm;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;

@Deprecated
public class CappingArm implements Controllable<Void> {

    private final Servo cappingServo;
    private static final double upPosition = 0.75;
    private static final double downPosition = 0.28;
    private static final double initPosition = 0.92;
    private static final double cappingPosition = 0.54;

    public CappingArm(HardwareMap hardwareMap, String servoName) {
        cappingServo = hardwareMap.servo.get(servoName);
        cappingServo.setPosition(initPosition);
    }

    public void setUpPosition() {
        cappingServo.setPosition(upPosition);
    }

    public void setDownPosition() {
        cappingServo.setPosition(downPosition);
    }

    public void setInitPosition() {
        cappingServo.setPosition(initPosition);
    }

    public void setToCappingPosition() {
        cappingServo.setPosition(cappingPosition);
    }

    /**
     * Allows control of a Capping Arm
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     * @return Always returns null
     */
    @Override
    public Void gamepadControl(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        if (gamepad2.x) {
            this.setDownPosition();
        } else if (gamepad2.b) {
            this.setUpPosition();
        } else if (gamepad2.a) {
            this.setToCappingPosition();
        }

        return null;
    }

    public void halt(){}

}
