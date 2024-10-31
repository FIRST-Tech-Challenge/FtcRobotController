package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Input;
import org.firstinspires.ftc.teamcode.Systems.Motors;

@TeleOp (name = "motor test")
public class testMotorMove extends LinearOpMode {

    Motors motors;

    @Override
    public void runOpMode() throws InterruptedException {
        motors = new Motors(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double leftsticky = gamepad1.left_stick_y * 100;
            double rightstickx = gamepad1.left_stick_y * 100;
            double leftstickx = gamepad1.left_stick_y * 100;


            Move(leftsticky);
            Spin(rightstickx * 100);
            Strafe(leftstickx * 100);

            telemetry.addData("MOVE:", "left_y (%.2f),", leftsticky);
            telemetry.addData("SPIN:", "right_x (%.2f),", rightstickx);
            telemetry.addData("Strafe:", "left_x (%.2f),", leftstickx);
            telemetry.update();
        }
    }

    private void Move(double power)
    {

        for (int i = 0; i < motors.driveTrainMotors.length; i++) {

            motors.MoveMotor(i, power); // loop over every motor and move them by gamepad input
        }
    }

    private void Strafe(double power)
    {
        motors.MoveMotor(1, power); // left front
        motors.MoveMotor(3, power); // right front

        motors.MoveMotor(0, -power); // left back
        motors.MoveMotor(2, -power); // right back
    }

    private void Spin(double power)
    {
        motors.MoveMotor(1, power); // left front
        motors.MoveMotor(0, power); // left back

        motors.MoveMotor(2, -power); // right front
        motors.MoveMotor(3, -power); // right back
    }
}
