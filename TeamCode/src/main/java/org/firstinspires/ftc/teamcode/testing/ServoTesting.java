package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Tester", group="Test Program")
public class ServoTesting  extends OpMode {
    public Servo testServo;
    @Override
    public void init() {
        testServo = hardwareMap.get(Servo.class, "TestServo");
    }

    @Override
    public void loop() {
        telemetry.addLine("Servo Tester");
        telemetry.addLine("============");
        telemetry.addData("Target position:", testServo.getPosition());
        telemetry.addLine("Use A to set position to 0");
        telemetry.addLine("Use B to set position to 1");
        telemetry.addLine("Use Dpad Left to set position to 0.1");
        telemetry.addLine("Use Dpad Right to set position to 0.9");

        if (gamepad1.a) testServo.setPosition(0);
        if (gamepad1.b) testServo.setPosition(1);
        if (gamepad1.dpad_left) testServo.setPosition(0.1);
        if (gamepad1.dpad_right) testServo.setPosition(0.9);
    }
}
