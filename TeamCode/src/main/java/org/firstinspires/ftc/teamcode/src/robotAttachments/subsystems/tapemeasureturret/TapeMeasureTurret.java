package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.tapemeasureturret;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;

public class TapeMeasureTurret implements Controllable<Void> {
    private final CRServo tapeMeasure;
    private final CRServo pitch;
    private final CRServo yaw;

    private double powerMultiplier = 1;

    public TapeMeasureTurret(HardwareMap hardwareMap, String tapeMeasureServoName, String pitchServoName, String yawServoName) {
        this.tapeMeasure = hardwareMap.crservo.get(tapeMeasureServoName);
        this.pitch = hardwareMap.crservo.get(pitchServoName);
        this.yaw = hardwareMap.crservo.get(yawServoName);
    }

    public void halt() {
        tapeMeasure.setPower(0);
        pitch.setPower(0);
        yaw.setPower(0);
    }

    /**
     * Allows control of a tape measure turret
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     */
    public Void gamepadControl(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        double pitchPower = gamepad2.left_stick_y * powerMultiplier;
        double yawPower = -gamepad2.right_stick_x * powerMultiplier;

        tapeMeasure.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        pitch.setPower(pitchPower);
        yaw.setPower(yawPower);

        if (gamepad2.a) {
            powerMultiplier = .1;
        }
        if (gamepad2.b) {
            powerMultiplier = .5;
        }
        if (gamepad2.y) {
            powerMultiplier = 1;
        }

        return null;

    }
}
