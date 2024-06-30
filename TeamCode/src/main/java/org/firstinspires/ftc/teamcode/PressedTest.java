package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class PressedTest extends LinearOpMode {
    //hardwaremap here
    private Servo servo0;
    @Override
    public void runOpMode() {
        servo0 = hardwareMap.get(Servo.class, "servo0");

        waitForStart();

        ButtonHandler buttonHandler = new ButtonHandler();

        if (isStopRequested()) return;

        boolean pressedLastIteration = false;
        double billy = 1.1;
        int bob = 1;


        while (opModeIsActive()) {

            boolean gamepad1A_pressed = gamepad1.a;
            boolean gamepad1B_pressed = gamepad1.b;

            if (buttonHandler.isPressedOnceA(gamepad1A_pressed)) {
                servo0.setPosition(0.7);
                telemetry.addData("A:", gamepad1A_pressed);
                telemetry.update();
            }
            if (buttonHandler.isPressedOnceB(gamepad1B_pressed)) {
                servo0.setPosition(0.2);
                telemetry.addData("B", gamepad1B_pressed);
                telemetry.update();
            }
        }

    }
}
