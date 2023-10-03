package org.firstinspires.ftc.teamcode.developingTestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Claw Test Code")
public class ClawTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo clawServo = hardwareMap.servo.get("claw");
        clawServo.setDirection(Servo.Direction.REVERSE);

        // sets servo's range and default position beforehand
        // WILL LIKELY NEED TO BE CHANGED AFTER TESTING


        waitForStart();

        // pressing button Y moves servo to hopefully open and then close the claw
        while (opModeIsActive()) {
            double clawServoPosition = clawServo.getPosition();
            float clawPos = 1f;

            if (gamepad1.right_bumper && clawPos <= 1 && clawPos <=0.1) {
                clawPos -= 0.1;
                clawServo.setPosition(clawPos);
            } else if (gamepad1.left_bumper && clawPos <= 0.9 && clawPos >= 0) {
                clawPos += 0.1;
                clawServo.setPosition(clawPos);
            }

            if (gamepad1.y) {
                clawServo.setPosition(0);
                sleep(2000);
                clawServo.setPosition(1);
            }

            // sends info about current servo position to driver station
            telemetry.addData("Claw Position: ", clawServoPosition);
            telemetry.update();

        }
    }
}