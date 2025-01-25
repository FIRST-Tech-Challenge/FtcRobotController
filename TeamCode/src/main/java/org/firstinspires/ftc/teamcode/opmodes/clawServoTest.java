package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "clawServoTest")
public class clawServoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "clawServo");

        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop
                if (gamepad1.y) {
                    servo.setPosition(0);

                } else if (gamepad1.x || gamepad1.b) {
                    servo.setPosition(0.5);

                } else if (gamepad1.a) {
                    servo.setPosition(1);
                }

                telemetry.addData("Status", "Running");
                telemetry.addData("position: ", servo.getPosition());
                telemetry.update();
            }
        }
    }
}
