package org.firstinspires.ftc.teamcode.core.robot;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * intake, extension of ToggleableTool
 */
public class Intake {
    private final GamepadEx toolGamepad;
    private final ToggleButtonReader reader;
    private final DcMotor motor;
    private final DistanceSensor distanceSensor;

    public Intake(@NonNull HardwareMap map, GamepadEx toolGamepad) {
        this.motor = map.get(DcMotor.class, "intake");
        this.reader = new ToggleButtonReader(toolGamepad, GamepadKeys.Button.X);
        this.distanceSensor = map.get(DistanceSensor.class, "intakeSensor");
        this.toolGamepad = toolGamepad;
    }

    public void update() {
        reader.readValue();
        if (distanceSensor.getDistance(DistanceUnit.MM) >= 210) {
            if (reader.getState()) {
                if (toolGamepad.getButton(GamepadKeys.Button.Y)) {
                    motor.setPower(1);
                } else {
                    motor.setPower(-1);
                }
            } else {
                motor.setPower(0);
            }
        } else if (toolGamepad.getButton(GamepadKeys.Button.Y)) {
            motor.setPower(1);
        } else {
            motor.setPower(0);
        }
    }
}
