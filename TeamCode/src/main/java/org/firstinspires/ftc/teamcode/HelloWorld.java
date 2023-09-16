package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name= "Hello World", group="Robot")
public class HelloWorld extends LinearOpMode {

    @Override
    public void runOpMode() {


        telemetry.addLine("Hello World");

        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.dpad_right) {
                telemetry.addLine("Right Button Has Been Pushed");

            }
            if (gamepad2.dpad_left) {
                telemetry.addLine("Left Button Has Been Pushed");
            }
            else{
                telemetry.addLine("Faulty Input");
            }
            telemetry.update();
        }

    }
}