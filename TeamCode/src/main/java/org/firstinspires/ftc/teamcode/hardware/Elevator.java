package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Elevator {
    public   double ServoPosition = 0;
    private Servo mainServo = null;
    public final double OPEN_POSITION_LEFT = 0.7;
    public final double OPEN_POSITION_RIGHT = 0.3;
    public Servo getLeftServo() {
        return leftServo;
    }
    public static boolean mainServoState = true; // tbrue = 1 false = 0
    public static boolean clawState = true; //true = open false = close
    private Servo leftServo = null;

    public Servo getRightServo() {
        return rightServo;
    }

    private Servo rightServo = null;
    private DcMotor elevatorMotor = null;

    private Telemetry telemetry;

    private static double mainServoPosition;
    private static double handsOffset;
    public static final double HANDS_MAX_POSITION = 0.5;
    public static final double HANDS_MIN_POSITION = -0.5;
    public static final double HANDS_MID_POSITION = 0;
    public static final double LEFT_RIGHT_SERVO_SPEED = 0.2;

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
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Starting at",  "%7d", elevatorMotor.getCurrentPosition());
        telemetry.update();

        leftServo.setPosition(OPEN_POSITION_LEFT);
        rightServo.setPosition(OPEN_POSITION_RIGHT);
        handsOffset = 0;
    }

    public void moveMainServo() {
        telemetry.addData("moveHands starts", true);
        telemetry.addData("mainservoState: ", mainServoState);
        if (mainServoState) {
            mainServo.setPosition(0);
            mainServoState = false;
            telemetry.addData("setPosition: ", 0);
        } else {
            mainServo.setPosition(1);
            mainServoState = true;
            telemetry.addData("setPosition: ", 1);
        }
        telemetry.update();
    }

    public void moveHands(){
        telemetry.addData("Servoposition", ServoPosition);
        if (clawState) {
            leftServo.setPosition(1);
            rightServo.setPosition(0);
            clawState = false;
        }
        else {
            leftServo.setPosition(OPEN_POSITION_LEFT);//OPEN_POSITION_LEFT);
            rightServo.setPosition(OPEN_POSITION_RIGHT);//OPEN_POSITION_RIGHT);
            clawState = true;
        }

        telemetry.update();
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

    public void moveHands(int direction) {

        handsOffset = handsOffset + direction * LEFT_RIGHT_SERVO_SPEED;
        handsOffset = Range.clip(handsOffset , HANDS_MIN_POSITION, HANDS_MAX_POSITION);
        leftServo.setPosition(HANDS_MID_POSITION + handsOffset);
        rightServo.setPosition(HANDS_MID_POSITION + handsOffset);

        telemetry.addData("left servo position: ", leftServo.getPosition());
        telemetry.addData("right servo position: ", rightServo.getPosition());
        telemetry.update();
    }
}
