package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class motorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "Left");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "Right");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        trim driveTrim = new trim();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            float leftThumbstickValue = gamepad1.left_stick_y;
            float rightThumbstickValue = gamepad1.right_stick_y;

            telemetry.addData("Left Thumbstick Value", leftThumbstickValue);
            telemetry.addData("Right Thumbstick Value", rightThumbstickValue);

            driveTrim.update();
            if (gamepad1.a) {
                leftMotor.setPower(-1 * driveTrim.trimLeft());
                //0.95 was the value that the trim was set to before I added my code
                rightMotor.setPower(1 * driveTrim.trimRight());
            } else {
                leftMotor.setPower(-0);
                rightMotor.setPower(0);

                if (gamepad1.b) {
                    leftMotor.setPower(1 * driveTrim.trimLeft());
                    rightMotor.setPower(-1 * driveTrim.trimRight());
                } else {
                    leftMotor.setPower(-0);
                    rightMotor.setPower(0);
                }
            }
            //leftMotor.setPower(leftThumbstickValue);
            //rightMotor.setPower(-rightThumbstickValue);
            telemetry.update();
        }
    }

}
