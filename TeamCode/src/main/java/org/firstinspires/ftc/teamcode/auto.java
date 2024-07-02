package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class auto extends OpMode {
    double cpr = 537.7;
    double gearRatio = 1;
    double diameter = 3.77;
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);
    double bias = 0.7;
    double conversion = cpi * bias;

    DcMotor frontLeftMotor; // location 0
    DcMotor frontRightMotor; // location 1
    DcMotor backLeftMotor; // location 3
    DcMotor backRightMotor; // location 2
    Servo servoFlag;

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        servoFlag = hardwareMap.get(Servo.class, "servoFlag");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        servoFlag.setPosition(0);

    }
    @Override
    public void start() {
        moveToPos(12, 0.2);
        servoFlag.setPosition(1);
    }

    @Override
    public void loop() {

    }
    @Override
    public void stop() {
    }

    public void moveToPos(double inches, double speed) {
        int move = (int)(Math.round(inches * conversion));

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + move);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + move);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + move);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + move);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

        while(frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()){

        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}