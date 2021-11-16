package org.firstinspires.ftc.teamcode.core.robot;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import androidx.annotation.NonNull;

/**
 * intake, extension of ToggleableTool
 */
public class Intake{
    private final GamepadEx toolGamepad;
    private final ButtonReader reader;
    private final DcMotor motor;
    private final DistanceSensor distanceSensor;
    public Intake(@NonNull HardwareMap map, GamepadEx toolGamepad) {
        this.motor = map.get(DcMotor.class, "intake");
        this.reader = new ButtonReader(toolGamepad, GamepadKeys.Button.X);
        this.distanceSensor = map.get(DistanceSensor.class, "intakeSensor");
        this.toolGamepad = toolGamepad;
    }
    public void run() {
        if (distanceSensor.getDistance(DistanceUnit.MM) >= 210) {
            reader.readValue();
            if (reader.wasJustReleased()) {
                if (toolGamepad.getButton(GamepadKeys.Button.Y)) {
                    motor.setPower(1);
                } else {
                    motor.setPower(-1);
                }
            } else {
                motor.setPower(0);
            }
        } else {
            motor.setPower(0);
        }
    }
}
