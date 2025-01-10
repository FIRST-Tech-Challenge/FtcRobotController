package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "LED Tester")
public class LEDTester extends LinearOpMode {
    private GamepadEvents controller;
    private LED led;
    @Override
    public void runOpMode() throws InterruptedException {
        led = new LED(hardwareMap, "led");
        controller = new GamepadEvents(gamepad1);
        waitForStart();
        while(opModeIsActive())
        {
            if(controller.a.onPress())
            {
                led.setBlue();
            }else if(controller.b.onPress())
            {
                led.setRed();
            }else if(controller.x.onPress())
            {
                led.setYellow();
            }else{
                led.setDefault();;
            }
        }
    }
}
