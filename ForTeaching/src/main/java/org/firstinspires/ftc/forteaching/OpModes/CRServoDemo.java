package org.firstinspires.ftc.forteaching.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.forteaching.CRServoCode;

@Disabled
@TeleOp(name = "CRServoDemo")
public class CRServoDemo extends OpMode {
    private CRServoCode servoCode;
    private CRServo servo;
    private Gamepad gamepad;

    @Override
    public void init() {
        // Called when INIT button being pressed
        this.servo = hardwareMap.get(CRServo.class, "gobilda_cr"); // need to match in the Robot Configuration
        this.servoCode = new CRServoCode(this.servo);
        this.gamepad = gamepad1;  // it's something built-in in the OpMode class
    }

    @Override
    public void start(){
        // Called when PLAY button being pressed
        this.servoCode.stop();
    }

    @Override
    public void loop() {
        // Called every time the robot runs
        if (this.gamepad.dpad_left) {
            this.servoCode.setMaxNegativeSpeed();
        } else if (this.gamepad.dpad_right) {
            this.servoCode.setMaxPositiveSpeed();
        } else if (this.gamepad.dpad_down) {
            this.servoCode.stop();
        }
    }
}
