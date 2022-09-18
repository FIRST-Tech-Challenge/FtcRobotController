package org.firstinspires.ftc.forteaching.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.forteaching.BasicServoCode;

@Disabled
@TeleOp(name = "ServoDirection00")
public class ServoDirection00 extends OpMode {
    private BasicServoCode servoCode;
    private Servo servo;

    @Override
    public void init() {
        // Called when INIT button being pressed
        this.servo = hardwareMap.get(Servo.class, "gobilda"); // need to match in the Robot Configuration
        this.servoCode = new BasicServoCode(this.servo);
    }

    @Override
    public void start(){
        // Called when PLAY button being pressed
        this.servoCode.setPosition(0.0);
    }

    @Override
    public void loop() {
        // No usage in this case
    }
}
