package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Elevator {
    private Servo mainServo = null;
    private Servo rightServo = null;
    private DcMotor elevatorMotor = null;
    private Telemetry telemetry;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry){
        mainServo = hardwareMap.get(Servo.class, "elevator_main_servo");
        rightServo = hardwareMap.get(Servo.class, "elevator_right_servo");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator_motor");
        this.telemetry = telemetry;
    }

    public void initMotors(){
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        mainServo.setPosition(Servo.MIN_POSITION);
        rightServo.setPosition(Servo.MAX_POSITION);
    }
    public void setElevatorMotorPower(double power){
        elevatorMotor.setPower(power);
        telemetry.addData("position", elevatorMotor.getCurrentPosition());
        telemetry.addData("terget position", elevatorMotor.getTargetPosition());
        telemetry.update();
    }

}
