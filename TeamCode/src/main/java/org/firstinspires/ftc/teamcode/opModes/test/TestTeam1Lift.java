package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

/**
 * Description: [Fill in]
 * Hardware:
 *  [motor0] Arm Motor
 *  [motor1] Unused
 *  [motor2] Unused
 *  [motor3] Unused
 *  [servo0] Unused
 *  [servo1] Unused
 *  [servo2] Unused
 *  [servo3] Unused
 * Controls:
 *  [Button] Function
 *
 */

@TeleOp(name="Test Team 1 Lift", group="Test")
//@Disabled
public class TestTeam1Lift extends TeleOpModeBase { // TODO: Test
    Motor motor;
    GamepadButton grabButton;
    boolean buttonPressed;
    boolean servoPosition;
    boolean lastPressed;
    // true if going to 120 or on 120, false if going to 240 or in 240
    @Override
    public void setup() {
        this.motor = HardwareMapContainer.motor0;
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setDistancePerPulse(0.015);
        new GamepadButton(
                Inputs.gamepad1, PSButtons.CIRCLE
        ).whenPressed(new InstantCommand(() -> {
            if (servoPosition){
                motor.setTargetDistance(18.0);
            }
            else{
                motor.setTargetDistance(0);
            }
            servoPosition = !servoPosition;
        }));
    }



    @Override
    public void every_tick() {
        buttonPressed = Inputs.gamepad1.getButton(GamepadKeys.Button.A);

        /**if (buttonPressed){
            if (!lastPressed){
                if (servoPosition){
                    motor.setTargetDistance(18.0);
                }
                else{
                    motor.setTargetDistance(0);
                }
                servoPosition = !servoPosition;
            }
            lastPressed = true;
        }
        else {
            lastPressed = false;
        }
         */

        motor.set(0.5);

        telemetry.addData("Status", "Running");
        telemetry.addData("Pressed", Inputs.gamepad1.getButton(GamepadKeys.Button.A));
        telemetry.update();
    }
}
