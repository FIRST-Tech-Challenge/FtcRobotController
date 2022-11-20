package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Elevator {
    private Servo mainServo = null;
    private Servo leftServo = null;
    private Servo rightServo = null;
    private DcMotor elevatorMotor = null;
    private Telemetry telemetry;
    private static double mainServoPosition;
    private static double leftServoPosition;
    private static double rightServoPosition;
    public static final double LEFT_RIGHT_SERVO_MAX_POSITION = 0.5;
    public static final double LEFT_RIGHT_SERVO_MIN_POSITION = -0.5;
    public static final double ELEVATOR_MOTOR_SPEED = 0.5;
    public static final double LEFT_RIGHT_SERVO_SPEED = 0.2;

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
        leftServo.setPosition(Servo.MIN_POSITION);
        rightServo.setPosition(Servo.MIN_POSITION);
        leftServoPosition = Servo.MIN_POSITION;
        rightServoPosition = Servo.MIN_POSITION;
    }
    public void setElevatorMotorPower(double power){
        elevatorMotor.setPower(power * ELEVATOR_MOTOR_SPEED);

        telemetry.addData("position", elevatorMotor.getCurrentPosition());
        telemetry.update();
    }

    public void leftRightServoPower(int direction) {
        double offset;
        double newLeftServoPosition;
        double newRightServoPosition;

        offset = direction * LEFT_RIGHT_SERVO_SPEED;
        newLeftServoPosition = leftServoPosition + offset;
        newRightServoPosition = rightServoPosition - offset;
        newLeftServoPosition = Range.clip(newLeftServoPosition, LEFT_RIGHT_SERVO_MIN_POSITION, LEFT_RIGHT_SERVO_MAX_POSITION);
        newRightServoPosition = Range.clip(newRightServoPosition, LEFT_RIGHT_SERVO_MIN_POSITION, LEFT_RIGHT_SERVO_MAX_POSITION);
        leftServo.setPosition(newLeftServoPosition);
        rightServo.setPosition(newRightServoPosition);
        leftServoPosition = leftServo.getPosition();
        rightServoPosition = rightServo.getPosition();

        telemetry.addData("left servo position: ", leftServoPosition);
        telemetry.addData("right servo position: ", rightServoPosition);
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
        //mainServo
    }
}
