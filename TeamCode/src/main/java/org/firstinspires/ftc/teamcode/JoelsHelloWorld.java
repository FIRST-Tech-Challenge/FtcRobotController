package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "Hello World", group="Robot")
public class JoelsHelloWorld extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Hello world");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.x) {
                telemetry.addData("gamepad1.x", gamepad1.x);
            }
            if (gamepad1.y) {
                telemetry.addData("gamepad1.y", gamepad1.y);
            }
            if (gamepad1.a) {
                telemetry.addData("gamepad1.a", gamepad1.a);
            }
            if (gamepad1.b) {
                telemetry.addData("gamepad1.b", gamepad1.b);
            }
        }
    }
}


