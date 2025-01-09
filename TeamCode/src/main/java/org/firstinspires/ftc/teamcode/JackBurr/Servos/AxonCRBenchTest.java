package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp
public class AxonCRBenchTest extends OpMode {
    public CRServo bottomLeft;
    public AnalogInput bottomLeftEncoder;
    public CRServo bottomRight;
    public AnalogInput bottomRightEncoder;
    public ElapsedTime timer = new ElapsedTime();
    public ServoController leftController;
    public ServoController rightController;
    @Override
    public void init() {
        bottomLeft = hardwareMap.get(CRServo.class, "bottom_left_diff");
        bottomLeftEncoder = hardwareMap.get(AnalogInput.class, "bottom_left_servo_encoder");
        bottomRight = hardwareMap.get(CRServo.class, "bottom_right_diff");
        bottomRightEncoder = hardwareMap.get(AnalogInput.class, "bottom_right_servo_encoder");
        leftController = bottomLeft.getController();
        rightController = bottomRight.getController();
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_left && timer.seconds() > 0.3){
            leftController.setServoPosition(bottomLeft.getPortNumber(), (getLeftServoPosition() - 0.05));
            rightController.setServoPosition(bottomRight.getPortNumber(), (getLeftServoPosition() - 0.05));
            timer.reset();
        }
        if(gamepad1.dpad_right && timer.seconds() > 0.3){
            leftController.setServoPosition(bottomLeft.getPortNumber(), (getLeftServoPosition() + 0.05));
            rightController.setServoPosition(bottomRight.getPortNumber(), (getLeftServoPosition() + 0.05));
            timer.reset();
        }
        telemetry.addData("Left Servo:", getLeftServoPosition());
        telemetry.addData("Right Servo:", getRightServoPosition());
        telemetry.addData("Left Encoder: ", String.valueOf(getLeftServoEncoderPosition()) + "°");
        telemetry.addData("Right Encoder: ", String.valueOf(getRightServoEncoderPosition()) + "°");
    }
    public double getLeftServoEncoderPosition(){
        return bottomLeftEncoder.getVoltage() / 3.3 * 360;
    }
    public double getRightServoEncoderPosition(){
        return bottomRightEncoder.getVoltage() / 3.3 * 360;
    }

    public double getLeftServoPosition(){
        return leftController.getServoPosition(bottomLeft.getPortNumber());
    }

    public double getRightServoPosition(){
        return rightController.getServoPosition(bottomRight.getPortNumber());
    }

}
