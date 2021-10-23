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
            double rearLeftPower = r * Math.sin(robotAngle) + (rx / 2);
            double frontLeftPower = r * Math.cos(robotAngle) + (rx / 2);
            double rearRightPower = r * Math.cos(robotAngle) - (rx / 2);
            double frontRightPower = r * Math.sin(robotAngle) - (rx / 2);
            robot.setFrontLeftDriveMotor(frontLeftPower);
            robot.setFrontRightDriveMotor(frontRightPower);
            robot.setRearLeftDriveMotor(rearLeftPower);
            robot.setFrontRightDriveMotor(rearRightPower);


        }
    }
}
