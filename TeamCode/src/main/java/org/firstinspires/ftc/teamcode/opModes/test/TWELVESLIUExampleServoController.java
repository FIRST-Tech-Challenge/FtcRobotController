package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;

/**
 * Description: [Fill in]
 * Hardware:
 *  [motor0] Left Drive Motor
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

@TeleOp(name="Servo Demo [12sliu]", group="Demo")
@Disabled
public class TWELVESLIUExampleServoController extends TeleOpModeBase {
    // TODO: Test
    ServoEx servoex;
    GamepadButton grabButton;
    boolean buttonPressed;
    boolean servoPosition;
    boolean lastPressed;
    // true if going to 120 or on 120, false if going to 240 or in 240
    @Override
    public void setup() {
        this.grabButton = new GamepadButton(
                Inputs.gamepad1, GamepadKeys.Button.A
        );

        // Initialise servo
        this.servoex = new SimpleServo(
                hardwareMap, "servo_name", 120, 240
        );
    }

    @Override
    public void every_tick() {
        buttonPressed = Inputs.gamepad1.getButton(GamepadKeys.Button.A);
        if (buttonPressed){
            if (!lastPressed){
                if (servoPosition){
                    servoex.turnToAngle(240);
                }
                else{
                    servoex.turnToAngle(120);
                }
                servoPosition = !servoPosition;
            }
            lastPressed = true;
        }
        else{
            lastPressed = false;
        }

        telemetry.addData("Status", "Running");
        telemetry.addData("Pressed", Inputs.gamepad1.getButton(GamepadKeys.Button.A));
        telemetry.update();
    }
}
