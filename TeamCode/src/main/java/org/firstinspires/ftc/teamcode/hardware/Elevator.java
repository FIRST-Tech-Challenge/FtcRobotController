package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Elevator {
    private final Servo leftServo;
    private final Servo rightServo;
    private final DcMotor elevatorMotor;
    private final Telemetry telemetry;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry) {
        leftServo = hardwareMap.get(Servo.class, "elevator_left_servo");
        rightServo = hardwareMap.get(Servo.class, "elevator_right_servo");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator_motor");
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.telemetry = telemetry;
    }

    public Elevator(LinearOpMode opMode) {
        leftServo = opMode.hardwareMap.get(Servo.class, "elevator_left_servo");
        rightServo = opMode.hardwareMap.get(Servo.class, "elevator_right_servo");
        elevatorMotor = opMode.hardwareMap.get(DcMotor.class, "elevator_motor");
        this.telemetry = opMode.telemetry;
        initMotors();
    }

    public void initMotors() {
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftServo.setPosition(Servo.MIN_POSITION);
        rightServo.setPosition(Servo.MAX_POSITION);
    }

    public void pinch() {
        //
    }

    public void setElevatorMotorPower(double power) {
        elevatorMotor.setPower(power);
        telemetry.addData("position", elevatorMotor.getCurrentPosition());
        telemetry.addData("target position", elevatorMotor.getTargetPosition());
        telemetry.addData("power", power);
        telemetry.addData("dcPower", elevatorMotor.getPower());
        telemetry.update();
    }

}
