package org.firstinspires.ftc.teamcode.opModes.test;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

/**
 * A program that controls a drive train has special wheels to go sideways.
 * Controls:
 *
 *  Left Joystick: Control the drive train in a normal way.
 *
 *  Square Toggle between normal speed and slow speed.
 *  Circle: Toggle between both directions and forward only.
 */

@Disabled
@TeleOp(name="Manual Drive Train [sbottingota]", group="Demo")
public class SBOTTINGOTASManualDriveTrain extends TeleOpModeBase {

    private Telemetry telemetry;

    private Motor leftMotor;
    private Motor rightMotor;

    private boolean isSlowMode = false;
    private boolean isForwardOnlyMode = false;

    private final GamepadEx gamepad = Inputs.gamepad1;

    @Override
    public void setup() {
        telemetry = TelemetryContainer.getTelemetry();

        leftMotor = HardwareMapContainer.motor0;
        rightMotor = HardwareMapContainer.motor1;

        new GamepadButton(gamepad, PSButtons.SQUARE).whenActive(() -> isSlowMode = !isSlowMode);
        new GamepadButton(gamepad, PSButtons.CIRCLE).whenActive(() -> isForwardOnlyMode = !isForwardOnlyMode);
    }

    private void arcadeDrive(@NonNull Motor leftMotor, @NonNull Motor rightMotor, double forwardSpeed, double turnSpeed) {
        leftMotor.set((forwardSpeed - turnSpeed) / 2D);
        rightMotor.set((forwardSpeed + turnSpeed) / 2D);
    }

    @Override
    public void every_tick() {

        final double[] inputs = {gamepad.getLeftY(), gamepad.getLeftX()}; //forward speed, turn speed, sideways speed

        //if joystick pos is less than this amount from in the middle, the robot doesn't move.
        final double DEAD_ZONE_SIZE = 0.2D;

        if (isSlowMode) {
            for (int i = 0; i < inputs.length; i++) {
                inputs[i] /= 2D;
            }
        }

        if (isForwardOnlyMode) {
            inputs[1] = 0D; //set turn speed to 0
        }

        //if the following variables are less than DEAD_ZONE_SIZE from 0, set them to be 0
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = Math.abs(inputs[i]) < DEAD_ZONE_SIZE ? inputs[i] : 0D;
        }

        arcadeDrive(leftMotor, rightMotor, inputs[0], inputs[1]);

        telemetry.addData("Normal Drive Train Y", inputs[0]);
        telemetry.addData("Normal Drive Train X", inputs[1]);

        telemetry.addData("Slow Mode", isSlowMode);
        telemetry.addData("Forward Only Mode", isForwardOnlyMode);
        telemetry.addData("Dead Zone Size", DEAD_ZONE_SIZE);
    }
}
