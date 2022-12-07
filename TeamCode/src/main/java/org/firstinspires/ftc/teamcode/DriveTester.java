package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriveTester extends OpMode {
    Motor fl;
    Motor fr;
    Motor rl;
    Motor rr;
    
    GamepadEx gamepadEx;
    
    @Override
    public void init() {
        fl = new Motor(hardwareMap, "front_left_drive");
        fr = new Motor(hardwareMap, "front_right_drive");
        rl = new Motor(hardwareMap, "rear_left_drive");
        rr = new Motor(hardwareMap, "rear_right_drive");
        
        gamepadEx = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        fl.set(gamepadEx.getButton(GamepadKeys.Button.DPAD_LEFT) ? .2 : 0);
        fr.set(gamepadEx.getButton(GamepadKeys.Button.DPAD_UP) ? .2 : 0);
        rl.set(gamepadEx.getButton(GamepadKeys.Button.DPAD_RIGHT) ? .2 : 0);
        rr.set(gamepadEx.getButton(GamepadKeys.Button.DPAD_DOWN) ? .2 : 0);
    }
}
