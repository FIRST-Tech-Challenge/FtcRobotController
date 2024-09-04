package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name="Testing Motors", group="TeleOp")

public class TestMotors extends OpMode {

    private boolean rightBumper, leftBumper = false;

    Robot robot = new Robot();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {
    }


    public void start() {

    }

    @Override
    public void loop() {



        // Test DriveTrain Motors
        if (gamepad1.x){
            robot.left_back.setPower(1);
        }
        else if (gamepad1.b){
            robot.right_front.setPower(1);
        }
        else if (gamepad1.y){
            robot.left_front.setPower(1);
        }
        else if (gamepad1.a){
            robot.right_back.setPower(1);
        }
        else{
            robot.brake();
        }

        telemetry.addData("Left Back", robot.left_back.getVelocity());
        telemetry.addData("Right Front", robot.right_front.getVelocity());
        telemetry.addData("Left Front", robot.left_front.getVelocity());
        telemetry.addData("Right Back", robot.right_back.getVelocity());

        //Test Arm and Claw
//        if(gamepad1.dpad_up) {
//            robot.arm.setVelocity(500);
//        } else if (gamepad1.dpad_down) {
//            robot.arm.setVelocity(-500);
//        } else {
//            robot.arm.setVelocity(0);
//        }
//
//        //telemetry.addData("Velocity", robot.left_front.getVelocity());
//        telemetry.addData("Arm Position,", robot.arm.getCurrentPosition());
//        //telemetry.addData("Claw Position", robot.claw.getPosition());
//        telemetry.addData("Claw Rotation", robot.claw_rot.getPosition());
//        telemetry.update();
    }
}