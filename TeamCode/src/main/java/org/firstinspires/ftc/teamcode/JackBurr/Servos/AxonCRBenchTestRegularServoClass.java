package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class AxonCRBenchTestRegularServoClass extends OpMode {
    public Servo bottomLeft;
    public AnalogInput bottomLeftEncoder;
    public Servo bottomRight;
    public AnalogInput bottomRightEncoder;
    public ElapsedTime timer = new ElapsedTime();
    public double leftTarget;
    public double rightTarget;
    @Override
    public void init() {
        bottomLeft = hardwareMap.get(Servo.class, "bottom_left_diff");
        bottomLeftEncoder = hardwareMap.get(AnalogInput.class, "bottom_left_servo_encoder");
        bottomRight = hardwareMap.get(Servo.class, "bottom_right_diff");
        bottomRightEncoder = hardwareMap.get(AnalogInput.class, "bottom_right_servo_encoder");
        leftTarget = getLeftServoEncoderPosition();
        rightTarget = getRightServoEncoderPosition();
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_left && timer.seconds() > 0.3){
            leftTarget = leftTarget - 0.05;
            rightTarget = rightTarget + 0.05;
            timer.reset();
        }
        if(gamepad1.dpad_right && timer.seconds() > 0.3){
            leftTarget = leftTarget + 0.05;
            rightTarget = rightTarget - 0.05;
            timer.reset();
        }
        if (getLeftServoEncoderPosition() < leftTarget) {
            bottomLeft.setPosition(1);
        }
        else if (getLeftServoEncoderPosition() > leftTarget){
            bottomLeft.setPosition(0);
        }
        else {
            bottomLeft.setPosition(0.5);
        }
        if (getRightServoEncoderPosition() < rightTarget) {
            bottomRight.setPosition(1);
        }
        else if (getLeftServoEncoderPosition() > rightTarget){
            bottomRight.setPosition(0);
        }
        else {
            bottomRight.setPosition(0.5);
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
