package org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ContinuousIntake {
    final static double forwardPower = 1;
    DcMotor intakeMotor;

    private static final double BucketUpPosition = .92;
    private static final double BucketDownPosition = .46;
    Servo slantServo;
    public ColorSensor intakeSensor;


    public ContinuousIntake(HardwareMap hardwareMap, String motorName, String servoName) {
        intakeMotor = hardwareMap.dcMotor.get(motorName);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slantServo = hardwareMap.servo.get(servoName);

    }

    /*
    This next constructor has a parameter for the name of the color sensor in the configuration
    and a boolean value for to have the color sensor LED on or off
     */
    public ContinuousIntake(HardwareMap hardwareMap, String motorName, String servoName, String colorSensor, Boolean sensorDetectionLight) {
        intakeMotor = hardwareMap.dcMotor.get(motorName);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeSensor = hardwareMap.colorSensor.get(colorSensor);

        intakeSensor.enableLed(sensorDetectionLight);
        slantServo = hardwareMap.servo.get(servoName);

    }

    public void intakeOn() {
        intakeMotor.setPower(forwardPower);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    public void intakeReverse() {
        intakeMotor.setPower(-forwardPower);
    }

    public void setMotorPower(double power) {
        intakeMotor.setPower(power);
    }

    public void setServoUp() {
        slantServo.setPosition(BucketUpPosition);
    }

    public void setServoDown() {
        slantServo.setPosition(BucketDownPosition);
    }


    /*
    this following method takes a parameter for the type of color and outputs the
    sensor's number for that color
     */
    public double getColor(String color) {
        int i;
        double[] colorNumber = {intakeSensor.red(), intakeSensor.blue(), intakeSensor.green()};
        switch (color) {
            case "red":
                i = 0;
                break;
            case "blue":
                i = 1;
                break;
            case "green":
                i = 2;
            default:
                i = 0;
        }
        return colorNumber[i];

    }

    public double[] getColor() {

        double[] colorNumber = {intakeSensor.red(), intakeSensor.green(), intakeSensor.blue()};
        return colorNumber;

    }
}
