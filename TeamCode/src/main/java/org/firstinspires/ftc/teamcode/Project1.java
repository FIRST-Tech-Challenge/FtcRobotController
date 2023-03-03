
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.lang.reflect.GenericArrayType;

@TeleOp (name="tommy")

public class Project1 extends LinearOpMode
{
    final static double ARM_HOME = 0.0;
    final static double ARM_MIN_RANGE = 0.0;
    final static double ARM_MAX_RANGE = 1.0;
    final static double HEXMOTOR_TPR = 288;
    final static double GEAR_REDUCTION = 1;
    final static double TICKS_PER_DEGREE = (HEXMOTOR_TPR * GEAR_REDUCTION) / 360;

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        MecanumDrive drivetrain = new MecanumDrive(robot);
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Orientation angles;
        Acceleration gravity;

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.arm.setTargetPosition(80);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.imu.resetYaw();
        drivetrain.MDrive(0,0,0,0);
        double target = 100;

        waitForStart();
        while (opModeIsActive()) {
            double vertical;
            double horizontal;
            double pivot;
            double heading = 0;
            PIDFCoefficients pidfNew = new PIDFCoefficients(3,0,0,0);
            PIDFCoefficients pidf = robot.arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("original:", "%.04f, %.04f, %.04f, %.04f", pidf.p, pidf.i, pidf.d, pidf.f);
            telemetry.addData("Encoder value", robot.arm.getCurrentPosition());
            telemetry.update();

            vertical   = gamepad1.left_stick_y;
            horizontal = -gamepad1.left_stick_x;
            pivot    =  gamepad1.right_stick_x;
            heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            drivetrain.MDrive(vertical, horizontal, pivot, heading);

            //if (gamepad1.x) robot.arm.setPower(0.4);
            //else if (gamepad1.y) robot.arm.setPower(-1);
            //else robot.arm.setPower(0);

            //if (gamepad1.dpad_down) robot.bucket.setPosition(0);
            //if (gamepad1.dpad_up) robot.bucket.setPosition(1);
            //if (gamepad1.dpad_left) robot.bucketAngle.setPosition(0.5);
            //if (gamepad1.dpad_right) robot.bucketAngle.setPosition(0);

            if (gamepad1.a) robot.clawAnglePosition += -0.001;
            if (gamepad1.b) robot.clawAnglePosition += 0.001;
            robot.clawAngle.setPosition(robot.clawAnglePosition);

            //if (gamepad1.left_bumper) robot.claw.setPosition(0.1);
            //if (gamepad1.right_bumper) claw.setPosition(-0.5);

            if (gamepad1.right_stick_button) robot.horz.setPower(-1);
            if (gamepad1.left_stick_button) robot.horz.setPower(0);

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) robot.imu.resetYaw();

            /*
            double Kp, Ki, Kd;
            double error = 0, lastError = 0, integralSum = 0;
            double out = 0;
            int cur = 0, reference = 30;
            ElapsedTime timer = new ElapsedTime();

            Kp = 0.005; Ki = 0; Kd = 0;


            if (gamepad1.right_bumper){
                timer.reset();
                while (1 == 1){
                    if (gamepad1.triangle){
                        reference += 10;
                        sleep(500);
                    }
                    cur = arm.getCurrentPosition();
                    telemetry.addData("Encoder value", arm.getCurrentPosition());
                    telemetry.update();
                    lastError = error;
                    error = reference-cur;
                    integralSum += error * timer.seconds();

                    //out = (error * Kp) + (integralSum * Ki) + ( ((error - lastError) / timer.seconds())  * Kd);

                    arm.setPower((error * Kp));

                    timer.reset();
                }

            }
            */ // custom PID



            robot.arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfNew);
            telemetry.addData("new", "%.04f, %.04f, %.04f, %.04f", pidfNew.p, pidfNew.i, pidfNew.d, pidfNew.f);



            /*if (gamepad1.right_bumper) {
                target = (target - 1);
            }
            */
            if (gamepad1.left_bumper) {
                target = (target + 1);
            }

            telemetry.addData("target", target);

            if (gamepad1.right_bumper){
                Pickup.pickup(100, 1000, 20, robot);
            }
            /*
            int arm_pos = robot.arm.getCurrentPosition();
            if (arm_pos <= 40){
                robot.arm.setPower(0.5);
            }
            else if (arm_pos <= target-10){
                robot.arm.setPower(0);
            }
            else if (arm_pos <= target+10){
                robot.arm.setPower(-0.3);
            }
            else{
                robot.arm.setPower(-0.8);
            }
            */

            /*
            if (gamepad1.left_bumper){
                robot.arm.setTargetPosition(0);
            }

            while (gamepad1.dpad_up || gamepad1.dpad_down){
                if (gamepad1.dpad_up) pidfNew.p = pidfNew.p + 0.1;
                else if (gamepad1.dpad_down) pidfNew.p = pidfNew.p - 0.1;
            }
            */


            // rev PID with setVelocity() and custom PIDF coefficients
            /*
            if (gamepad1.right_bumper){
                robot.arm.setTargetPosition((int)(TICKS_PER_DEGREE * 100));
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.5);
            }
            */
        }
    }
}