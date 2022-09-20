package org.firstinspires.ftc.teamcode.Prototyping;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Prototype extends LinearOpMode {
    ServoEx servo;
    GamepadEx gp;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = new SimpleServo(hardwareMap, "claw_servo", 0, 90);
        gp = new GamepadEx(gamepad1);

        while(opModeIsActive() && !isStopRequested()){
            if(gp.getButton(GamepadKeys.Button.A)){
                servo.turnToAngle(90);
            }

            if(gp.getButton(GamepadKeys.Button.B)){
                servo.turnToAngle(0);
            }
        }
    }
}
