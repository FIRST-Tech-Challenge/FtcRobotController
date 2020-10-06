package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum_Control_V2", group="Testier")

public class MecanumControlV2 extends OpMode {

    MecanumDrive robot  = new MecanumDrive();

    double driveSpeed;
    double turnSpeed;
    double direction;

    private ElapsedTime period  = new ElapsedTime();
    private double runtime = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);

        msStuckDetectInit = 18000;
        msStuckDetectLoop = 18000;

        telemetry.addData("Hello","be ready");
        telemetry.addData("Loop_Timeout",msStuckDetectLoop);
        telemetry.update();
    }
    @Override
    public void loop() {
        //telemetry.addData("Drive Speed",driveSpeed);
        //telemetry.addData("Direction",direction);
        //telemetry.addData("Turn Speed", turnSpeed);
        //telemetry.addData("LB",robot.getLBencoder());
        //telemetry.addData("RB",robot.getRBencoder());
        //telemetry.addData("LF",robot.getLFencoder());
        //telemetry.addData("RF",robot.getRFencoder());
        telemetry.update();

        //Speed control (turbo/slow mode) and direction of stick calculation
        if (gamepad1.left_bumper){
            driveSpeed = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            direction = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            turnSpeed = gamepad1.right_stick_x;
        }else if (gamepad1.right_bumper){
            driveSpeed = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y)*.4;
            direction = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            turnSpeed = gamepad1.right_stick_x*.4;
        }else {
            driveSpeed = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y)*.7;
            direction = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            turnSpeed = gamepad1.right_stick_x*.7;
        }
        //set power and direction of drive motors
        if (turnSpeed == 0){
            robot.MecanumController(driveSpeed,direction,0);
        }
        //control of turning
        if (gamepad1.right_stick_x != 0 && driveSpeed == 0){
            robot.leftFront.setPower(turnSpeed);
            robot.leftBack.setPower(turnSpeed);
            robot.rightFront.setPower(-turnSpeed);
            robot.rightBack.setPower(-turnSpeed);
        }
    }
}
