package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotClass;

@TeleOp
@Disabled
public class EncoderTest extends LinearOpMode {

    RobotClass robot = new RobotClass(this);

    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                robot.resetEncoders();
            } else if (gamepad1.b) {
                robot.frontLeft.setTargetPosition(1000);
                robot.frontRight.setTargetPosition(1000);
                robot.backLeft.setTargetPosition(1000);
                robot.backRight.setTargetPosition(1000);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.25);
                robot.frontRight.setPower(0.25);
                robot.backLeft.setPower(0.25);
                robot.backRight.setPower(0.25);

                while (robot.backRight.isBusy()) {
                    telemetry.addData("Front Left", robot.frontLeft.getCurrentPosition());
                    telemetry.addData("Front Right", robot.frontRight.getCurrentPosition());
                    telemetry.addData("Back Left", robot.backLeft.getCurrentPosition());
                    telemetry.addData("Back Right", robot.backRight.getCurrentPosition());

                    telemetry.update();
                }

                robot.stopMotors();
            }

            telemetry.addData("Front Left", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", robot.frontRight.getCurrentPosition());
            telemetry.addData("Back Left", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right", robot.backRight.getCurrentPosition());

            telemetry.update();
        }
    }
}
