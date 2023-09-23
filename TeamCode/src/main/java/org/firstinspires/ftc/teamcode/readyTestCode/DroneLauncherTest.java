package org.firstinspires.ftc.teamcode.readyTestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drone Launcher Test Code")
public class DroneLauncherTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo droneServo = hardwareMap.servo.get("drone");
        droneServo.setDirection(Servo.Direction.FORWARD);

        // sets servo's range and default position beforehand
        // WILL LIKELY NEED TO BE CHANGED AFTER TESTING
        droneServo.scaleRange(0, 0.25);
        droneServo.setPosition(0);
        double droneServoPosition = droneServo.getPosition();

        waitForStart();

        // pressing button A moves servo to hopefully launch the drone
        // pressing button Y resets the servo's position to default to reload the rubber band
        while (opModeIsActive()) {
            if (gamepad1.a) {
                droneServo.setPosition(1);
            }else if (gamepad1.y) {
                droneServo.setPosition(0);
            }

            // sends info about current servo position to driver station
            telemetry.addData("Servo Position: ", droneServoPosition);
            telemetry.update();

        }
    }
}