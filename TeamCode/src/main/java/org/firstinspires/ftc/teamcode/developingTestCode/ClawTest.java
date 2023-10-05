package org.firstinspires.ftc.teamcode.developingTestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Lift+Wrist Test Code")
public class ClawTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotor.Direction.FORWARD);

        Servo wristServo = hardwareMap.servo.get("wrist");
        wristServo.setDirection(Servo.Direction.REVERSE);

        // sets servo's range and default position beforehand
        // WILL LIKELY NEED TO BE CHANGED AFTER TESTING


        waitForStart();

        // pressing button Y moves servo to hopefully open and then close the claw
        while (opModeIsActive()) {
            double wristServoPosition = wristServo.getPosition();
            float wristPos = 1f;

            double rTrigger = gamepad1.right_trigger/3;
            double lTrigger = gamepad1.left_trigger/3;

            // holding down right trigger makes lift move up
            // and left trigger makes lift move down
            // depending on how much you press
            // while you aren't pressing anything, sets the motor power to zero
            if (rTrigger > 0) {
                lift.setPower(rTrigger);
            }else if (lTrigger > 0) {
                lift.setPower(-lTrigger);
            }else {
                lift.setPower(0);
            }

            if (gamepad1.right_bumper && wristPos <= 1 && wristPos <=0.1) {
                telemetry.addLine("right_bumper clawPos -= 1");
                telemetry.update();
                wristPos -= 1;
                wristServo.setPosition(wristPos);
            } else if (gamepad1.left_bumper && wristPos <= 0.9 && wristPos >= 0) {
                telemetry.addLine("left_bumper clawPos += 1");
                telemetry.update();
                wristPos += 1;
                wristServo.setPosition(wristPos);
            }

            if (gamepad1.y) {
                wristServo.setPosition(0);
                sleep(2000);
                wristServo.setPosition(1);
            }

            // sends info about current servo position to driver station
            telemetry.addData("Claw Position: ", wristServoPosition);
            telemetry.addData("ClawPos var: ", wristPos);
            telemetry.update();

        }
    }
}