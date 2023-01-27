package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp(name="Servo Demo", group="Demo")

// no clue what the thing up there is for
public class ExampleServoController extends TeleOpModeBase {

    ServoEx servoex;
    GamepadEx gamepadEx;
    GamepadButton grabButton;
    boolean buttonPressed;
    boolean servoPosition;
    boolean lastPressed;
    // true if going to 120 or on 120, false if going to 240 or in 240
    @Override
    public void setup() {
        this.gamepadEx = new GamepadEx(gamepad1);
        this.grabButton = new GamepadButton(
                gamepadEx, GamepadKeys.Button.A
        );

        // Initialise servo
        this.servoex = new SimpleServo(
                hardwareMap, "servo_name", 120, 240
        );
    }

    @Override
    public void every_tick() {
        buttonPressed = gamepadEx.getButton(GamepadKeys.Button.A);
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
        telemetry.addData("Pressed", gamepadEx.getButton(GamepadKeys.Button.A));
        telemetry.update();
    }
}
