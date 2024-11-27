package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Input;
import org.firstinspires.ftc.teamcode.Systems.Motors;

@TeleOp (name = "motor test")
public class testMotorMove extends LinearOpMode {

    Motors motors;
    Input input;

    @Override
    public void runOpMode() throws InterruptedException {
        motors = new Motors(hardwareMap);
        input = new Input(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            double leftsticky = gamepad1.left_stick_y * 100;
            double rightstickx = gamepad1.right_stick_x * 100;
            double leftstickx = gamepad1.left_stick_x * 100;


            input.move(leftsticky);
            input.spin(rightstickx);
            input.strafe(leftstickx);

            telemetry.addData("MOVE:", "left_y (%.2f),", leftsticky);
            telemetry.addData("SPIN:", "right_x (%.2f),", rightstickx);
            telemetry.addData("STRAFE:", "left_x (%.2f),", leftstickx);
            telemetry.update(); // telemtryy
        }
    }
}
