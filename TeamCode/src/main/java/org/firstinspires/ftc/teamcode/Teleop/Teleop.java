package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, timer);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) { // clearer nomenclature for variables
            double ly = gamepad1.left_stick_y;
            double lx = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double ry = gamepad1.right_stick_y;

            double r = Math.hypot(lx, ly);
            double robotAngle = Math.atan2(ly, lx) - Math.PI / 4;
            double lrPower = r * Math.sin(robotAngle) + rx;
            double lfPower = r * Math.cos(robotAngle) + rx;
            double rrPower = r * Math.cos(robotAngle) - rx;
            double rfPower = r * Math.sin(robotAngle) - rx;
            frontLeft.setPower(lfPower);
            frontRight.setPower(rfPower);
            backLeft.setPower(lrPower);
            backRight.setPower(rrPower);


        }
    }
}
