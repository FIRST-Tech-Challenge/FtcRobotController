package org.firstinspires.ftc.teamcode.readyTestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drone Launcher Test Code")
@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class DroneLauncherTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo droneServo = hardwareMap.servo.get("drone");
        droneServo.setDirection(Servo.Direction.FORWARD);

        // sets servo's range and default position beforehand
        // WILL LIKELY NEED TO BE CHANGED AFTER TESTING
        droneServo.scaleRange(0, 1);
        droneServo.setPosition(0.85);

        waitForStart();

        // pressing button A moves servo to hopefully launch the drone and then reset launcher position
        while (opModeIsActive()) {
            double droneServoPosition = droneServo.getPosition();
            if (gamepad1.a) {
                droneServo.setPosition(1);
                sleep(1500);
                droneServo.setPosition(0.85);
            }

            // sends info about current servo position to driver station
            telemetry.addData("Servo Position: ", droneServoPosition);
            telemetry.update();

        }
    }
}