package org.firstinspires.ftc.teamcode.developingTestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drone Test 2")
public class DroneTest2 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo droneServo = hardwareMap.servo.get("drone");
        droneServo.setDirection(Servo.Direction.FORWARD);

        // sets servo's range and default position beforehand
        // WILL LIKELY NEED TO BE CHANGED AFTER TESTING
        droneServo.scaleRange(0, 1);
        droneServo.setPosition(0.85);
        sleep(500);
        droneServo.setPosition(1);
        waitForStart();

        // pressing button A moves servo to hopefully launch the drone
        // pressing button Y resets the servo's position to default to reload the rubber band


    }
}