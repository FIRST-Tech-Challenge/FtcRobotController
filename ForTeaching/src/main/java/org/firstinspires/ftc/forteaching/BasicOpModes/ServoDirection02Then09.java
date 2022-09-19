package org.firstinspires.ftc.forteaching.BasicOpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.forteaching.BasicServoCode;

@Disabled
@TeleOp(name = "ServoDirection02Then09", group = "demo")
public class ServoDirection02Then09 extends OpMode {
    private BasicServoCode servoCode;
    private Servo servo;

    @Override
    public void init() {
        // Called when INIT button being pressed
        this.servo = hardwareMap.get(Servo.class, "gobilda"); // need to match in the Robot Configuration
        this.servoCode = new BasicServoCode(this.servo);
    }

    @Override
    public void start() {
        // Called when PLAY button being pressed
        this.servoCode.setPosition(0.2);
        this.servoCode.setPosition(0.9);
    }

    @Override
    public void loop() {
        // No usage in this case
    }
}
