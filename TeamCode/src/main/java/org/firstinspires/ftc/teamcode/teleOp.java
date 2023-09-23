package org.firstinspires.ftc.teamcode;

//By Ethan Clawsie and Aman Sulaiman, 2021-2022 Freight Frenzy4

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import java.awt.font.NumericShaper;

@TeleOp(group = "DreamMachines", name = "FrenzyOpModeTest")
public class teleOp extends LinearOpMode {
    double cascadeMotorPower;

    public piRhosHardware robot = new piRhosHardware();

    public void runOpMode() {
        robot.initTeleOpIMU(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            cascadeMotorPower = Range.clip(cascadeMotorPower, .0, .7);
            robot.frontRight.setPower(-0.5 * ((gamepad1.left_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x));
            robot.frontLeft.setPower(-0.5 * ((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x));
            robot.backRight.setPower(-0.5 * ((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x));
            robot.backLeft.setPower(-0.5 * ((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x));
            robot.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            if (gamepad1.right_trigger > 0.5){
                robot.carouselpowerfortime(.5, .1);
            }
            if (gamepad1.left_trigger > 0.5){
                robot.carouselpowerfortime(-.5, .1);
            }

            if (gamepad2.y) {

                cascadeMotorPower += .2;
                robot.cascadeMotor1.setPower(cascadeMotorPower);
                robot.cascadeMotor2.setPower(cascadeMotorPower);

            }
            if (gamepad2.a && !(robot.touchRight.isPressed() || robot.touchLeft.isPressed())) {

                cascadeMotorPower -= .2;
                robot.cascadeMotor1.setPower(cascadeMotorPower);
                robot.cascadeMotor2.setPower(cascadeMotorPower);

            } else {
                robot.cascadeMotor1.setPower(0);
                robot.cascadeMotor2.setPower(0);
            }

            /*if (gamepad2.dpad_left) {
                robot.slideForTime(0.5, 0.05);
            }

            if (gamepad2.dpad_right) {
                robot.slideForTime(-0.5, 0.05);
            }*/

            /*if (gamepad2.left_bumper) {
                robot.intake1.setPower(1);
                robot.intake2.setPower(-1);
            }
            if (gamepad2.right_bumper) {
                robot.intake1.setPower(-1);
                robot.intake2.setPower(1);
            }
            if (!gamepad2.left_bumper) {
                robot.intake1.setPower(0);
                robot.intake2.setPower(0);
            }
            if (!gamepad2.right_bumper) {
                robot.intake1.setPower(0);
                robot.intake2.setPower(0);
            }*/
            /*if (!gamepad2.right_bumper && !gamepad2.left_bumper){
                robot.intake2.setPower(0);
                robot.intake1.setPower(0);
            }
            if(gamepad2.right_bumper){
                robot.intake1.setPower(-.75);
                robot.intake2.setPower(.75);
            }
            if (gamepad2.left_bumper) {
                robot.intake1.setPower(.75);
                robot.intake2.setPower(-.75);
            }*/


            if (gamepad2.x){
                robot.bucket.setPosition(.2);
            }
            if (gamepad2.b){
                robot.bucket.setPosition(0);
            }
            if (gamepad2.dpad_up){
                robot.bucket.setPosition(.3);
            }
        }
    }
}