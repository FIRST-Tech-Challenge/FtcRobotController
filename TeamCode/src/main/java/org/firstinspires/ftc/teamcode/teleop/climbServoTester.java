package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ClimbServo;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

public class climbServoTester extends LinearOpMode {
    GamepadEvents controller;
    ClimbServo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = new ClimbServo(hardwareMap, "leftServo", "rightServo");
        controller = new GamepadEvents(gamepad1);
        waitForStart();
        while(opModeIsActive())
        {
            if(controller.dpad_left.onPress())
            {
                servo.openServo();
            }else if(controller.dpad_right.onPress())
            {
                servo.closeServo();
            }
        }
    }
}
