package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class AxonPickupTest extends OpMode {
    public Servo grippers;
    public Servo left_servo;
    public AnalogInput left_servo_encoder;
    public AnalogInput right_servo_encoder;
    public Servo right_servo;
    public ElapsedTime button_timer = new ElapsedTime();
    public enum ElbowState {
        DOWN,
        UP
    }
    public enum GripperState {
        OPEN,
        CLOSED
    }
    public ElbowState elbowState = ElbowState.DOWN;
    public GripperState gripperState = GripperState.OPEN;
    public int GRIPPERS_CLOSED = 1;
    public int GRIPPERS_OPEN = 0;

    @Override
    public void init() {
        grippers = hardwareMap.get(Servo.class, "grippers");
        left_servo = hardwareMap.get(Servo.class, "left_diff");
        right_servo = hardwareMap.get(Servo.class, "right_diff");
        left_servo_encoder = hardwareMap.get(AnalogInput.class,"left_servo_encoder");
        right_servo_encoder = hardwareMap.get(AnalogInput.class,"right_servo_encoder");
        left_servo.setPosition(1);
        right_servo.setPosition(1);
        grippers.setPosition(1);
    }

    @Override
    public void loop() {
        if(gamepad1.x && button_timer.seconds() > 0.3){
            switch (elbowState){
                case UP:
                    elbowState = ElbowState.DOWN;
                    break;
                case DOWN:
                    grippers.setPosition(GRIPPERS_CLOSED);
                    gripperState = GripperState.CLOSED;
                    elbowState = ElbowState.UP;
            }
            button_timer.reset();
        }
        if(gripperState == GripperState.OPEN){
            grippers.setPosition(GRIPPERS_OPEN);
        }
        else {
            grippers.setPosition(GRIPPERS_CLOSED);
        }
        if(elbowState == ElbowState.UP){
            if(grippers.getPosition() == 1) {
                left_servo.setPosition(0.1);
                right_servo.setPosition(0.1);
            }
        }
        else {
            left_servo.setPosition(1);
            right_servo.setPosition(1);
        }
        telemetry.addData("Left Servo Position:",left_servo.getPosition());
        telemetry.addData("Right Servo Position:",right_servo.getPosition());
        telemetry.addData("Left Encoder: ", String.valueOf(left_servo_encoder.getVoltage() / 3.3 * 360) + "°");
        telemetry.addData("Right Encoder: ", String.valueOf(right_servo_encoder.getVoltage() / 3.3 * 360) + "°");
        telemetry.addData("Servo position:", grippers.getPosition());
    }
}
