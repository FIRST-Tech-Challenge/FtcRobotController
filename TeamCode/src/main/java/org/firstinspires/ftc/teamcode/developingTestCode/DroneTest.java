package org.firstinspires.ftc.teamcode.developingTestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drone Test")
public class DroneTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo droneServo = hardwareMap.servo.get("drone");
        droneServo.setDirection(Servo.Direction.FORWARD);

        // sets servo's range and default position beforehand
        // WILL LIKELY NEED TO BE CHANGED AFTER TESTING
        droneServo.scaleRange(0, 1);
        droneServo.setPosition(0);
        float dronePosition = 0f;
        waitForStart();

        // pressing button A moves servo to hopefully launch the drone
        // pressing button Y resets the servo's position to default to reload the rubber band
        while (opModeIsActive()) {
            telemetry.addData("Pos var: ", dronePosition);
            telemetry.update();
            if (gamepad1.a) {
                if (dronePosition <= 0.9 && dronePosition >= 0) {
                    dronePosition += 0.1;
                }
                droneServo.setPosition(dronePosition);
                telemetry.addData("Pos + =", dronePosition);
                telemetry.update();
            }

            if (gamepad1.y) {
                if (dronePosition <= 1 && dronePosition >= 0.1) {
                    dronePosition -= 0.1;
                }
                droneServo.setPosition(dronePosition);
                telemetry.addData("Pos - =", dronePosition);
                telemetry.update();
                // sends info about current servo position to driver station
            }


        }

    }
}