package org.firstinspires.ftc.forteaching.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.forteaching.BasicServoCode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BasicServoDemo")
public class BasicServoDemo extends OpMode {
    private BasicServoCode servoCode;
    private Servo servo;
    private Gamepad gamepad;

    @Override
    public void init() {
        // Called when INIT button being pressed
        this.servo = hardwareMap.get(Servo.class, "gobilda"); // need to match in the Robot Configuration
        this.servoCode = new BasicServoCode(this.servo);
        this.gamepad = gamepad1;  // it's something built-in in the OpMode class
    }

    @Override
    public void start(){
        // Called when PLAY button being pressed
        // this.servoCode.toLeft();
        this.servoCode.toMiddle();
    }

    @Override
    public void loop() {
        // Called every time the robot runs
        // Might be a bad solution since this will set the servo to the same position multiple times
        // in the duration of the button being pressed
        if (this.gamepad.dpad_left) {
            this.servoCode.toLeft();
        } else if (this.gamepad.dpad_right) {
            this.servoCode.toRight();
        } else if (this.gamepad.dpad_up) {
            this.servoCode.toMiddle();
        }
    }
}
