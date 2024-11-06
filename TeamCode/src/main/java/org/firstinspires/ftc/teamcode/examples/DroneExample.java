package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DroneExample extends LinearOpMode {
    private CRServo droneServo;
    @Override
    public void runOpMode(){
        droneServo = hardwareMap.get(CRServo.class, "droneServo");

        telemetry.addData("Robot", "initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.x) {
                droneServo.setPower(0.3); // Rotate the servo a half rotation (0.5)
                sleep(750);
                droneServo.setPower(0); // Set servo position back to 0
            }
        }
    }
}
