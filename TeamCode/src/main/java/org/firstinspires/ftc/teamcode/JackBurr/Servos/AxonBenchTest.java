package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp
public class AxonBenchTest extends OpMode {
    public Servo bottomLeft;
    public AnalogInput bottomLeftEncoder;
    public Servo bottomRight;
    public AnalogInput bottomRightEncoder;
    public ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        bottomLeft = hardwareMap.get(Servo.class, "bottom_left_diff");
        bottomLeftEncoder = hardwareMap.get(AnalogInput.class, "bottom_left_servo_encoder");
        bottomRight = hardwareMap.get(Servo.class, "bottom_right_diff");
        bottomRightEncoder = hardwareMap.get(AnalogInput.class, "bottom_right_servo_encoder");
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_left && timer.seconds() > 0.3){
            bottomLeft.setPosition(bottomLeft.getPosition() - 0.05);
            bottomRight.setPosition(bottomRight.getPosition() - 0.05);
            timer.reset();
        }
        if(gamepad1.dpad_right && timer.seconds() > 0.3){
            bottomLeft.setPosition(bottomLeft.getPosition() + 0.05);
            bottomRight.setPosition(bottomRight.getPosition() + 0.05);
            timer.reset();
        }
        telemetry.addData("Left Servo:", bottomLeft.getPosition());
        telemetry.addData("Right Servo:", bottomRight.getPosition());
        telemetry.addData("Left Encoder: ", String.valueOf(getLeftServoEncoderPosition()) + "°");
        telemetry.addData("Right Encoder: ", String.valueOf(getRightServoEncoderPosition()) + "°");
    }
    public double getLeftServoEncoderPosition(){
        return bottomLeftEncoder.getVoltage() / 3.3 * 360;
    }
    public double getRightServoEncoderPosition(){
        return bottomRightEncoder.getVoltage() / 3.3 * 360;
    }
}
