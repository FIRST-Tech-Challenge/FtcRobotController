package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Elevator {
    private final Servo mainServo;
    private final Servo leftServo;
    private final Servo rightServo;
    private final DcMotor elevatorMotor;

    private final Telemetry telemetry;

    private static double mainServoPosition;
    public static final double HANDS_OPEN_POSITION = 0.5;
    public static final double HANDS_CLOSED_POSITION = 0.1;
    public static final double HANDS_MID_POSITION = 0;

    public static final double ELEVATOR_MOTOR_SPEED = 0.5;
    public static final int MAX_ELEVATOR_POSITION = 30;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry){
        mainServo = hardwareMap.get(Servo.class, "elevator_main_servo");
        leftServo = hardwareMap.get(Servo.class, "elevator_left_servo");
        rightServo = hardwareMap.get(Servo.class, "elevator_right_servo");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator_motor");
        this.telemetry = telemetry;
    }

    public void initMotors(){
        mainServo.setPosition(Servo.MIN_POSITION);
        mainServoPosition = Servo.MIN_POSITION;

        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        //move elevator down - start position
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Starting at",  "%7d", elevatorMotor.getCurrentPosition());
        telemetry.update();

        leftServo.setPosition(HANDS_MID_POSITION);
        rightServo.setPosition(HANDS_MID_POSITION);
    }

    public void setElevatorMotorPower(double power){
        int currentPosition = elevatorMotor.getCurrentPosition();
        if (currentPosition >= MAX_ELEVATOR_POSITION || currentPosition<= 0){
            elevatorMotor.setPower(0);
        }
        else {
            elevatorMotor.setPower(power * ELEVATOR_MOTOR_SPEED);
        }

        telemetry.addData("position", elevatorMotor.getCurrentPosition());
        telemetry.update();
    }

     private void moveHands(double position) {

        leftServo.setPosition(position);
        rightServo.setPosition(position);

        telemetry.addData("left servo position: ", leftServo.getPosition());
        telemetry.addData("right servo position: ", rightServo.getPosition());
        telemetry.update();
    }

    public void moveMainServo() {
        mainServoPosition += 1;
        if (mainServoPosition >1 ) {
            mainServoPosition = 0;
        }
        mainServo.setPosition(mainServoPosition);
    }

    public void stop() {
        // stop mainServo
    }

    public void openHands() {
        moveHands(HANDS_OPEN_POSITION);
    }

    public void closeHands() {
        moveHands(HANDS_CLOSED_POSITION);
    }
}
