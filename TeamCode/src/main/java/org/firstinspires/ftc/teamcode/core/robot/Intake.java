package org.firstinspires.ftc.teamcode.core.robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.thread.thread.EventThread;
import androidx.annotation.NonNull;

/**
 * intake, extension of ToggleableTool
 */
public class Intake extends ToggleableTool<DcMotor> {
    private final GamepadEx toolGamepad;
    private final DistanceSensor distanceSensor;
    public Intake(EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad) {
        super(eventThread, map, toolGamepad, DcMotor.class, "intake", GamepadKeys.Button.X, -1);
        this.distanceSensor = map.get(DistanceSensor.class, "intakeSensor");
        this.toolGamepad = toolGamepad;
    }
    @Override
    protected void run() {
        if (distanceSensor.getDistance(DistanceUnit.MM) >= 20) {
            if (reader.getState()) {
                motor.setPower(power);
            } else if (toolGamepad.getButton(GamepadKeys.Button.Y)) {
                motor.setPower(-power);
            } else {
                motor.setPower(0);
            }
        } else {
            motor.setPower(0);
        }
    }
}
