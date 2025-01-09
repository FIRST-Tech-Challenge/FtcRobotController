package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class AxonPickupTest3 extends OpMode {
    public DiffConstantsV1 diffConstantsV1 = new DiffConstantsV1();
    public Servo grippers;
    public Servo left_servo;
    public AnalogInput left_servo_encoder;
    public AnalogInput right_servo_encoder;
    public Servo right_servo;
    public ElapsedTime button_timer = new ElapsedTime();
    public ElapsedTime servoTimer = new ElapsedTime();
    public ElapsedTime elbowTimer = new ElapsedTime();
    public enum ElbowState {
        DOWN,
        UP
    }
    public enum GripperState {
        OPEN,
        CLOSED
    }

    //Note: below the left/right aren't actual left/right
    public enum ElbowRangeState {
        LEFT_OF_RANGE,
        IN_RANGE,
        RIGHT_OF_RANGE
    }

    public ElbowState elbowState = ElbowState.UP;
    public GripperState gripperState = GripperState.OPEN;
    public int GRIPPERS_CLOSED = 1;
    public int GRIPPERS_OPEN = 0;
    //TODO: Adjust values
    public double LEFT_SERVO_ZERO = diffConstantsV1.LEFT_SERVO_UP;
    public double LEFT_SERVO_ONE = diffConstantsV1.LEFT_SERVO_DOWN;
    public double RIGHT_SERVO_ZERO = diffConstantsV1.RIGHT_SERVO_UP;
    public double RIGHT_SERVO_ONE = diffConstantsV1.RIGHT_SERVO_DOWN;

    public double RIGHT_SERVO_POSITION = 1;
    public double LEFT_SERVO_POSITION = 1;

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
        if (gamepad1.x && button_timer.seconds() > 0.3) {
            switch (elbowState) {
                case UP:
                    elbowState = ElbowState.DOWN;
                    break;
                case DOWN:
                    servoTimer.reset();
                    grippers.setPosition(GRIPPERS_CLOSED);
                    gripperState = GripperState.CLOSED;
                    elbowState = ElbowState.UP;
            }
            button_timer.reset();
        }
        if (gripperState == GripperState.OPEN) {
            grippers.setPosition(GRIPPERS_OPEN);
        } else {
            grippers.setPosition(GRIPPERS_CLOSED);
        }
        if (elbowState == ElbowState.UP) {
            if(servoTimer.seconds() > 0.3) {
                if(elbowTimer.seconds() > 1){
                    runLeftServoToEncoderPos(LEFT_SERVO_ZERO);
                    runRightServoToEncoderPos(RIGHT_SERVO_ZERO);
                }
            }
        }
        else {
                runLeftServoToEncoderPos(LEFT_SERVO_ONE);
                runRightServoToEncoderPos(RIGHT_SERVO_ONE);
        }
        telemetry.addData("Left encoder: ", getLeftServoEncoderPosition());
        telemetry.addData("Right encoder: ", getRightServoEncoderPosition());

    }

    public void runLeftServoToEncoderPos(double position){
        double encoder_pos = getLeftServoEncoderPosition();
        ElbowRangeState elbowRangeState = getElbowRangeState(encoder_pos, position, 0);
        if (elbowRangeState == ElbowRangeState.IN_RANGE){
            telemetry.addLine("Left servo is in range.");
        }
        else if (elbowRangeState == ElbowRangeState.LEFT_OF_RANGE){
            telemetry.addLine("Left Servo is attempting to go up from " + encoder_pos + " to " + position);
            telemetry.addLine("Left: " + LEFT_SERVO_POSITION + " to " + (LEFT_SERVO_POSITION - 0.1));
            LEFT_SERVO_POSITION = setLeftServoPosition(LEFT_SERVO_POSITION - 0.01);
        }
        else {
            telemetry.addLine("Left Servo is attempting to go down from " + encoder_pos + " to " + position);
            telemetry.addLine("Left: " + LEFT_SERVO_POSITION + " to " + (LEFT_SERVO_POSITION + 0.1));
            LEFT_SERVO_POSITION = setLeftServoPosition(LEFT_SERVO_POSITION + 0.01);
        }
    }

    public void runRightServoToEncoderPos(double position){
        double encoder_pos = getRightServoEncoderPosition();
        ElbowRangeState elbowRangeState = getElbowRangeState(encoder_pos, position, 0);
        if (elbowRangeState == ElbowRangeState.IN_RANGE){
            telemetry.addLine("Right servo is in range.");
        }
        else if (elbowRangeState == ElbowRangeState.LEFT_OF_RANGE){
            telemetry.addLine("Right Servo is attempting to go down from " + encoder_pos + " to " + position);
            telemetry.addLine("Right: " + RIGHT_SERVO_POSITION + " to " + (RIGHT_SERVO_POSITION + 0.1));
            //TODO: Adjust these
            RIGHT_SERVO_POSITION = setRightServoPosition(RIGHT_SERVO_POSITION - 0.01);
        }
        else {
            telemetry.addLine("Right Servo is attempting to go up from " + encoder_pos + " to " + position);
            telemetry.addLine("Right: " + RIGHT_SERVO_POSITION + " to " + (RIGHT_SERVO_POSITION - 0.1));
            RIGHT_SERVO_POSITION = setRightServoPosition(RIGHT_SERVO_POSITION  + 0.01);
        }
    }

    public double setLeftServoPosition(double position){
        if(position > 1){
            position = 1;
        }
        else if (position < 0){
            position = 0;
        }
        left_servo.setPosition(position);
        return position;
    }

    public double setRightServoPosition(double position){
        if(position > 1){
            position = 1;
        }
        else if (position < 0){
            position = 0;
        }
        right_servo.setPosition(position);
        return position;
    }
    public double getLeftServoEncoderPosition(){
        return left_servo_encoder.getVoltage() / 3.3 * 360;
    }
    public double getRightServoEncoderPosition(){
        return right_servo_encoder.getVoltage() / 3.3 * 360;
    }

    public ElbowRangeState getElbowRangeState(double number, double target_number, double difference){
        double left_number = target_number - difference;
        double right_number = target_number + difference;
        if(number > left_number && number < right_number) {
            return ElbowRangeState.IN_RANGE;
        }
        else if(number == left_number || number == right_number) {
            return ElbowRangeState.IN_RANGE;
        }
        else if (number < left_number){
            //Raise encoder pos
            return ElbowRangeState.LEFT_OF_RANGE;
        }
        else {
            //Lower encoder pos
            return ElbowRangeState.RIGHT_OF_RANGE;
        }
    }

}
