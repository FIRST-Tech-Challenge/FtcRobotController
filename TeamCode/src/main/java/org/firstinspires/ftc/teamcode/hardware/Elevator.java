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
    public final double CLOSE_POSITION_LEFT = 1;
    public final double CLOSE_POSITION_RIGHT = 0.5;

    public final double OPEN_POSITION_LEFT = 0.8;
    public final double OPEN_POSITION_RIGHT = 0.6;
    public Servo getLeftServo() {
        return leftServo;
    }
    public static boolean mainServoState = true; // trrue = 1 false = 0
    public static boolean clawState = false; //true = open false = close
    private Servo leftServo = null;

    public Servo getRightServo() {
        return rightServo;
    }

    private Servo rightServo = null;
    public DcMotor elevatorMotor = null;

    private Telemetry telemetry;


    public final int firstFloor = -1658;
    public final int secondFloor = -2837;
    public final int thirdFloor = -3967;
    public final int groundFloor = 0;

    public int currentFloor = groundFloor;

    private static double mainServoPosition;
    private static double handsOffset;
    public static final double HANDS_MAX_POSITION = 0.5;
    public static final double HANDS_MIN_POSITION = -0.5;
    public static final double HANDS_MID_POSITION = 0;
    public static final double LEFT_RIGHT_SERVO_SPEED = 0.2;

    public static final double ELEVATOR_MOTOR_SPEED = 0.5;
    public static final int MAX_ELEVATOR_POSITION = 4800;

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
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Starting at",  "%7d", elevatorMotor.getCurrentPosition());


        leftServo.setPosition(CLOSE_POSITION_LEFT);
        rightServo.setPosition(CLOSE_POSITION_RIGHT);
        telemetry.addData("startPosition", elevatorMotor.getCurrentPosition());
        telemetry.update();
        handsOffset = 0;
    }

    public void moveMainServo() {
        telemetry.addData("moveHands starts", true);
        telemetry.addData("mainservoState: ", mainServoState);
        if (mainServoState) {
            mainServo.setPosition(1);
            mainServoState = false;
            telemetry.addData("setPosition: ", 1);
        } else {
            mainServo.setPosition(0);
            mainServoState = true;
            telemetry.addData("setPosition: ", 0);
        }
        //telemetry.update();
    }

    public void moveHands(){
        telemetry.addData("Servoposition", ServoPosition);
        if (clawState) {
            leftServo.setPosition(1);
            rightServo.setPosition(0.5);
            clawState = false;
        }
        else {
            leftServo.setPosition(OPEN_POSITION_LEFT);//OPEN_POSITION_LEFT);
            rightServo.setPosition(OPEN_POSITION_RIGHT);//OPEN_POSITION_RIGHT);
            clawState = true;
        }

    //    telemetry.update();
    }

    public void setElevatorMotorPower(double power) {
        int currentPosition = elevatorMotor.getCurrentPosition();
        telemetry.addData("position", currentPosition);
        telemetry.addData("power in function", power);
        if ((currentPosition >= MAX_ELEVATOR_POSITION && power > 0) || (currentPosition < 0 && power < 0)) {
            elevatorMotor.setPower(0);
        } else {
            elevatorMotor.setPower(power * ELEVATOR_MOTOR_SPEED);
        }
//        if (currentPosition <0 && power<0)
//            elevatorMotor.setPower(0);
//        telemetry.addData("position", currentPosition);
    }
    public void setElevatorPosition(int elevetorPosition){
        currentFloor = elevetorPosition;
        elevatorMotor.setTargetPosition(elevetorPosition);
        elevatorMotor.setPower(ELEVATOR_MOTOR_SPEED);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

public void setElevetorDown(){
        elevatorMotor.setTargetPosition(currentFloor - 400);
    }
}
