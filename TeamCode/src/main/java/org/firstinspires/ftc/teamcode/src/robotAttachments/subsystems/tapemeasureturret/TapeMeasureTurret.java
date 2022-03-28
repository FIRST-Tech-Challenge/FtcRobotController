package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.tapemeasureturret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;

public class TapeMeasureTurret implements Controllable {
    private final CRServo tapeMeasure;
    private final CRServo pitch;
    private final CRServo yaw;

    private double powerMultiplier = 1;

    public TapeMeasureTurret(HardwareMap hardwareMap, String tapeMeasureServoName, String pitchServoName, String yawServoName) {
        this.tapeMeasure = hardwareMap.crservo.get(tapeMeasureServoName);
        this.pitch = hardwareMap.crservo.get(pitchServoName);
        this.yaw = hardwareMap.crservo.get(yawServoName);
    }

    /**
     * Allows control of a tape measure turret
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     */
    public Object gamepadControl(Gamepad gamepad1, Gamepad gamepad2) {
        double pitchPower = gamepad2.left_stick_y * powerMultiplier;
        double yawPower = -gamepad2.right_stick_x * powerMultiplier;

        tapeMeasure.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        pitch.setPower(pitchPower);
        yaw.setPower(yawPower);

        if (gamepad2.a){
            powerMultiplier = .1;
        }
        if (gamepad2.b){
            powerMultiplier = .5;
        }
        if (gamepad2.y){
            powerMultiplier = 1;
        }

        return null;

    }
}
