package org.firstinspires.ftc.teamcode.TestBot;

//By Ethan Clawsie and Aman Sulaiman, 2021-2022 Freight Frenzy4

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(group = "PiRhos", name = "bussin2")
public class TestTeleOp extends LinearOpMode {
    double cascadeMotorPower;

    public TestHardware robot = new TestHardware();

    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            cascadeMotorPower = Range.clip(cascadeMotorPower, -.1, .8);
            robot.frontRight.setPower(.8 * ((gamepad1.left_stick_y - gamepad1.right_stick_x) + gamepad1.left_stick_x));
            robot.frontLeft.setPower(.8 * ((gamepad1.left_stick_y - gamepad1.right_stick_x) - gamepad1.left_stick_x));
            robot.backRight.setPower(.8 * ((gamepad1.left_stick_y + gamepad1.right_stick_x) - gamepad1.left_stick_x));
            robot.backLeft.setPower(.8 * ((gamepad1.left_stick_y + gamepad1.right_stick_x) + gamepad1.left_stick_x));
            //robot.cascadeMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




            //robot.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);



            /*if (gamepad1.right_trigger > 0.5){
                robot.carouselpowerfortime(.5, .1);
            }
            if (gamepad1.left_trigger > 0.5){
                robot.carouselpowerfortime(-.5, .1);
            }*/


            /*if (gamepad2.a) {

                if (robot.touchLeft.isPressed() || robot.touchRight.isPressed()){
                    robot.cascadeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    robot.cascadeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    robot.cascadeMotorRight.setPower(0);
                    robot.cascadeMotorLeft.setPower(0);
                    robot.timer.reset();
                    robot.cascadeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.cascadeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                else{
                    cascadeMotorPower = -.4;
                    robot.cascadeMotorRight.setPower(-cascadeMotorPower);
                    robot.cascadeMotorLeft.setPower(cascadeMotorPower);

                }

            }
            if (gamepad2.y) {
                while(gamepad2.y) {
                    cascadeMotorPower += .2;
                    robot.cascadeMotorRight.setPower(-cascadeMotorPower);
                    robot.cascadeMotorLeft.setPower(cascadeMotorPower);
                }
                robot.cascadeMotorRight.setPower(.10);
                robot.cascadeMotorLeft.setPower(.10);

            } else {
                robot.cascadeMotorRight.setPower(0);
                robot.cascadeMotorLeft.setPower(0);
            }
            /*if (gamepad2.right_trigger > .3){
                robot.intake2.setPower(1);
                robot.intake1.setPower(-1);
            }
            if (gamepad2.left_trigger > .3){
                robot.intake2.setPower(-1);
                robot.intake1.setPower(2);
            }
            if (gamepad2.left_trigger < .3 && gamepad2.right_trigger < .3){
                robot.intake1.setPower(0);
                robot.intake2.setPower(0);
            }*/
        }
    }
}