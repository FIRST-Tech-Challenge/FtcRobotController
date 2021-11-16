package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@TeleOp(name="Teleop w/ test controls")
public class teleop extends LinearOpMode{
    zanehardware robot = new zanehardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //wait for start
        waitForStart();
        while (opModeIsActive()){
            robot.allMotorPower(gamepad1.left_stick_y);

        }
    }
    public void turn(){
        if (gamepad1.left_stick_x > 0){

            robot.Front_Left.setPower(Math.abs(gamepad1.left_stick_x));
            robot.Back_Left.setPower(Math.abs(gamepad1.left_stick_x));
            robot.Front_Right.setPower(-Math.abs(gamepad1.left_stick_x));
            robot.Back_Right.setPower(-Math.abs(gamepad1.left_stick_x));
        } else if (gamepad1.left_stick_x < 0){

            robot.Front_Left.setPower(Math.abs(gamepad1.left_stick_x));
            robot.Back_Left.setPower(Math.abs(gamepad1.left_stick_x));
            robot.Front_Right.setPower(-Math.abs(gamepad1.left_stick_x));
            robot.Back_Right.setPower(-Math.abs(gamepad1.left_stick_x));
        }
    }
}
