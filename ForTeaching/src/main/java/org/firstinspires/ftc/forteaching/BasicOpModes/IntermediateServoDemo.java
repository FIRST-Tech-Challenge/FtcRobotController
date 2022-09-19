package org.firstinspires.ftc.forteaching.BasicOpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.forteaching.BasicServoCode;
import org.firstinspires.ftc.forteaching.util.MathHelper;

@Disabled
@TeleOp(name = "IntermediateServoDemo", group = "demo")
public class IntermediateServoDemo extends OpMode {
    private BasicServoCode servoCode;
    private Servo servo;
    private Gamepad gamepad;
    private double currentServoPosition = 0.0;
    private boolean isButtonJustPressed = true;

    @Override
    public void init() {
        // Called when INIT button being pressed
        this.servo = hardwareMap.get(Servo.class, "gobilda"); // need to match in the Robot Configuration
        this.servoCode = new BasicServoCode(this.servo);
        this.gamepad = gamepad1;  // it's something built-in in the OpMode class
    }

    @Override
    public void start() {
        // Called when PLAY button being pressed
        // this.servoCode.toLeft();
        this.servoCode.toMiddle();
    }

    @Override
    public void loop() {
        // Called every time the robot runs
        // Might be a bad solution since this will set the servo to the same position multiple times
        // in the duration of the button being set the position multiple times
        // UPDATE: its fine to keep setting the servo to the same position multiple times
        if (this.gamepad.dpad_left) {
            this.servoCode.toLeft();
        } else if (this.gamepad.dpad_right) {
            this.servoCode.toRight();
        } else if (this.gamepad.dpad_up) {
            this.servoCode.toMiddle();
        } else if (this.gamepad.left_bumper) {
            if (!this.isButtonJustPressed) {
                this.isButtonJustPressed = true;
                this.currentServoPosition = MathHelper.clamp(this.currentServoPosition - 0.05, 0.0, 1.0);
                this.servoCode.setPosition(this.currentServoPosition);
            }
        } else if (this.gamepad.right_bumper) {
            if (!this.isButtonJustPressed) {
                this.isButtonJustPressed = true;
                this.currentServoPosition = MathHelper.clamp(this.currentServoPosition + 0.05, 0.0, 1.0);
                this.servoCode.setPosition(this.currentServoPosition);
            }
        } else {
            this.isButtonJustPressed = false;
        }

        telemetry.addData("IsButtonJustPressed", isButtonJustPressed);
        telemetry.addData("CurrentServoPosition", currentServoPosition);
        telemetry.addData("left_bumper", this.gamepad.left_bumper);
        telemetry.addData("right_bumper", this.gamepad.right_bumper);
    }
}
