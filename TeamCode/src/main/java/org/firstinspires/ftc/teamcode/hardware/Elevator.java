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

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry){
        mainServo = hardwareMap.get(Servo.class, "elevator_main_servo");
        leftServo = hardwareMap.get(Servo.class, "elevator_left_servo");
        rightServo = hardwareMap.get(Servo.class, "elevator_right_servo");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator_motor");
        this.telemetry = telemetry;
    }

    public void initMotors(){
        mainServo.setPosition(Servo.MIN_POSITION);
        mainServoPosition = 0;
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        leftServo.setPosition(Servo.MIN_POSITION);
        rightServo.setPosition(Servo.MAX_POSITION);
    }
    public void setElevatorMotorPower(double power){
//        elevatorMotor.setPower(power);
//        telemetry.addData("position", elevatorMotor.getCurrentPosition());
//        telemetry.addData("terget position", elevatorMotor.getTargetPosition());
//        telemetry.update();
    }

    public void leftRightServoPower(int direction) {
//        double clawOffset = 0;
//        final double MID_SERVO   =  0.5 ;
//        final double CLAW_SPEED  = 0.02 ;
//        clawOffset = clawOffset + direction * CLAW_SPEED;
//        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//        leftServo.setPosition(MID_SERVO + clawOffset);
//        rightServo.setPosition(MID_SERVO - clawOffset);

    }

    public void mainServoPower() {
//        mainServoPosition += 1;
//        if (mainServoPosition >1 )
//            mainServoPosition = 0;
//        mainServo.setPosition(mainServoPosition);
    }

    public void stop() {
        //mainServo
    }
}
