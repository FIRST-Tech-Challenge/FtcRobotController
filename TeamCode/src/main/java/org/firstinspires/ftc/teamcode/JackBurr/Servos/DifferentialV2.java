package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DifferentialV2 {
    public DiffConstantsV2 diffConstantsV2= new DiffConstantsV2();
    public Servo topLeft;
    public AnalogInput topLeftEncoder;
    public Servo topRight;
    public AnalogInput topRightEncoder;
    public int ENCODER_DIFFERENCE = 50;
    public Telemetry telemetry = null;
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

    public double TOP_LEFT_SERVO_POSITION = 1;
    public double TOP_RIGHT_SERVO_POSITION = 1;
    //TODO: Change the below to add #s
    public double BOTTOM_LEFT_SERVO_POSITION;
    public double BOTTOM_RIGHT_SERVO_POSITION;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.topLeft = hardwareMap.get(Servo.class, "left_diff");
        this.topLeftEncoder = hardwareMap.get(AnalogInput.class, "left_servo_encoder");
        this.topRight = hardwareMap.get(Servo.class, "right_diff");
        this.topRightEncoder = hardwareMap.get(AnalogInput.class,"right_servo_encoder");
        this.telemetry = telemetry;
    }

    public void runTopLeftServoToEncoderPos(double position) {
        double encoder_pos = getTopLeftServoEncoderPosition();
        ElbowRangeState elbowRangeState = getElbowRangeState(encoder_pos, position, ENCODER_DIFFERENCE);
        if (elbowRangeState == ElbowRangeState.IN_RANGE) {
            telemetry.addLine("Top Left servo is in range.");
        } else if (elbowRangeState == ElbowRangeState.LEFT_OF_RANGE) {
            telemetry.addLine("Top Left Servo is attempting to go up from " + encoder_pos + " to " + position);
            telemetry.addLine("Top Left: " + TOP_LEFT_SERVO_POSITION + " to " + (TOP_LEFT_SERVO_POSITION - 0.01));
            TOP_LEFT_SERVO_POSITION = setTopLeftServoPosition(TOP_LEFT_SERVO_POSITION + 0.01);
        } else {
            telemetry.addLine("Top Left Servo is attempting to go down from " + encoder_pos + " to " + position);
            telemetry.addLine("Top Left: " + TOP_LEFT_SERVO_POSITION + " to " + (TOP_LEFT_SERVO_POSITION + 0.01));
            TOP_LEFT_SERVO_POSITION = setTopLeftServoPosition(TOP_LEFT_SERVO_POSITION - 0.01);
        }
    }

    /*
    public void runBottomLeftServoToEncoderPos(double position) {
        double encoder_pos = getBottomLeftServoEncoderPosition();
        ElbowRangeState elbowRangeState = getElbowRangeState(encoder_pos, position, ENCODER_DIFFERENCE);
        if (elbowRangeState == ElbowRangeState.IN_RANGE) {
            telemetry.addLine("Bottom Left servo is in range.");
        } else if (elbowRangeState == ElbowRangeState.LEFT_OF_RANGE) {
            telemetry.addLine("Bottom Left Servo is attempting to go up from " + encoder_pos + " to " + position);
            telemetry.addLine("Bottom Left: " + BOTTOM_LEFT_SERVO_POSITION + " to " + (BOTTOM_LEFT_SERVO_POSITION - 0.01));
            //TODO: Switch these signs if needed
            BOTTOM_LEFT_SERVO_POSITION = setBottomLeftServoPosition(BOTTOM_LEFT_SERVO_POSITION - 0.01);
        } else {
            telemetry.addLine("Bottom Left Servo is attempting to go down from " + encoder_pos + " to " + position);
            telemetry.addLine("Bottom Left: " + BOTTOM_LEFT_SERVO_POSITION + " to " + (BOTTOM_LEFT_SERVO_POSITION + 0.01));
            BOTTOM_LEFT_SERVO_POSITION = setBottomLeftServoPosition(TOP_LEFT_SERVO_POSITION + 0.01);
        }
    }

     */

    public void runTopRightServoToEncoderPos(double position){
        double encoder_pos = getTopRightServoEncoderPosition();
        ElbowRangeState elbowRangeState = getElbowRangeState(encoder_pos, position, ENCODER_DIFFERENCE);
        if (elbowRangeState == ElbowRangeState.IN_RANGE){
            telemetry.addLine("Top Right servo is in range.");
        }
        else if (elbowRangeState == ElbowRangeState.LEFT_OF_RANGE){
            telemetry.addLine("Top Right Servo is attempting to go down from " + encoder_pos + " to " + position);
            telemetry.addLine("Top Right: " + TOP_RIGHT_SERVO_POSITION + " to " + (TOP_RIGHT_SERVO_POSITION - 0.01));
            TOP_RIGHT_SERVO_POSITION = setTopRightServoPosition(TOP_RIGHT_SERVO_POSITION - 0.01);
        }
        else {
            telemetry.addLine("Top Right Servo is attempting to go up from " + encoder_pos + " to " + position);
            telemetry.addLine("Top Right: " + TOP_RIGHT_SERVO_POSITION + " to " + (TOP_RIGHT_SERVO_POSITION + 0.01));
            TOP_RIGHT_SERVO_POSITION = setTopRightServoPosition(TOP_RIGHT_SERVO_POSITION  + 0.01);
        }
    }

    /*
    public void runBottomRightServoToEncoderPos(double position){
        double encoder_pos = getBottomRightServoEncoderPosition();
        ElbowRangeState elbowRangeState = getElbowRangeState(encoder_pos, position, ENCODER_DIFFERENCE);
        if (elbowRangeState == ElbowRangeState.IN_RANGE){
            telemetry.addLine("Bottom Right servo is in range.");
        }
        else if (elbowRangeState == ElbowRangeState.LEFT_OF_RANGE){
            telemetry.addLine("Bottom Right Servo is attempting to go down from " + encoder_pos + " to " + position);
            telemetry.addLine("Bottom Right: " + BOTTOM_RIGHT_SERVO_POSITION + " to " + (BOTTOM_RIGHT_SERVO_POSITION - 0.01));
            //TODO: Switch these signs if needed
            BOTTOM_RIGHT_SERVO_POSITION = setBottomRightServoPosition(BOTTOM_RIGHT_SERVO_POSITION - 0.01);
        }
        else {
            telemetry.addLine("Bottom Right Servo is attempting to go up from " + encoder_pos + " to " + position);
            telemetry.addLine("Bottom Right: " + BOTTOM_RIGHT_SERVO_POSITION + " to " + (BOTTOM_RIGHT_SERVO_POSITION + 0.01));
            BOTTOM_RIGHT_SERVO_POSITION = setBottomRightServoPosition(BOTTOM_RIGHT_SERVO_POSITION  + 0.01);
        }
    }
     */

    /*
    public double setBottomLeftServoPosition(double position){
        if(position > 1){
            position = 1;
        }
        else if (position < 0){
            position = 0;
        }
        bottomLeft.setPosition(position);
        return position;
    }
     */

    public double setTopLeftServoPosition(double position){
        if(position > 1){
            position = 1;
        }
        else if (position < 0){
            position = 0;
        }
        topLeft.setPosition(position);
        return position;
    }

    /*
    public double setBottomRightServoPosition(double position){
        if(position > 1){
            position = 1;
        }
        else if (position < 0){
            position = 0;
        }
        bottomRight.setPosition(position);
        return position;
    }
     */
    public double setTopRightServoPosition(double position){
        if(position > 1){
            position = 1;
        }
        else if (position < 0){
            position = 0;
        }
        topRight.setPosition(position);
        return position;
    }

    public double getTopLeftServoEncoderPosition(){
        return topLeftEncoder.getVoltage() / 3.3 * 360;
    }

    public boolean leftEncoderIsPast(double position){
        return getTopLeftServoEncoderPosition() < position;
    }
    /*
    public double getBottomLeftServoEncoderPosition(){
        return bottomLeftEncoder.getVoltage() / 3.3 * 360;
    }
     */
    public double getTopRightServoEncoderPosition(){
        return topRightEncoder.getVoltage() / 3.3 * 360;
    }
    public boolean rightEncoderIsPast(double position){
        return getTopRightServoEncoderPosition() > position;
    }
    /*
    public double getBottomRightServoEncoderPosition(){
        return bottomRightEncoder.getVoltage() / 3.3 * 360;
    }
     */
    public double getTopRightServoPosition(){
        return TOP_RIGHT_SERVO_POSITION;
    }
    public double getTopLeftServoPosition(){
        return TOP_LEFT_SERVO_POSITION;
    }
    public double getBottomRightServoPosition(){
        return BOTTOM_RIGHT_SERVO_POSITION;
    }
    public double getBottomLeftServoPosition(){
        return BOTTOM_LEFT_SERVO_POSITION;
    }

    public void topServosUp(boolean debug){
        setTopLeftServoPosition(0.85);
        setTopRightServoPosition(0.85);
        if(debug){
            telemetry.addData("Left Servo Position: ", topLeft.getPosition());
            telemetry.addData("Right Servo Position: ", topRight.getPosition());
        }
    }
    public void topServosDown(boolean debug){
        setTopLeftServoPosition(0);
        setTopRightServoPosition(0);
        if(debug){
            telemetry.addData("Left Servo Position: ", topLeft.getPosition());
            telemetry.addData("Right Servo Position: ", topRight.getPosition());
        }

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
