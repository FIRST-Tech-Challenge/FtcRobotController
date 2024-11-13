package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class motorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //DcMotor leftMotor = hardwareMap.get(DcMotor.class, "Left");
        //DcMotor rightMotor = hardwareMap.get(DcMotor.class, "Right");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        boolean isDpadLeft =false,isDpadRight=false;
        Trim t = new Trim();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("left Trim", t.getLeftTrim());
            telemetry.addData("right Trim", t.getRightTrim());

            float leftThumbstickValue = gamepad1.left_stick_y;
            float rightThumbstickValue = gamepad1.right_stick_y;
            PowerLevels pl=t.getPowerLevel(leftThumbstickValue,rightThumbstickValue);
            telemetry.addData("Left Thumbstick Value", leftThumbstickValue);
            telemetry.addData("Right Thumbstick Value", rightThumbstickValue);

            if (gamepad1.dpad_left && !isDpadLeft){
                t.addLeft();
                isDpadLeft = true;
            }

            if (!gamepad1.dpad_left){
                isDpadLeft = false;
            }

            
            if (gamepad1.dpad_right){
                t.addRight();
            }

            /*if (gamepad1.a) {
                leftMotor.setPower(-pl.getLeftPower());
                //0.95 was the value that the Trim was set to before I added my code
                rightMotor.setPower(pl.getRightPower());
            } else {
                leftMotor.setPower(-0);
                rightMotor.setPower(0);

                if (gamepad1.b) {
                    leftMotor.setPower(pl.getLeftPower());
                    rightMotor.setPower(-pl.getRightPower());
                } else {
                    leftMotor.setPower(-0);
                    rightMotor.setPower(0);
                }
            }*/
            //leftMotor.setPower(leftThumbstickValue);
            //rightMotor.setPower(-rightThumbstickValue);
            telemetry.update();
        }
    }

}
