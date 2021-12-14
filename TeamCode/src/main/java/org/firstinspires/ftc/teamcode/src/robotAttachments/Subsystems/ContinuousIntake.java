package org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems;


import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ContinuousIntake {
    final static double forwardPower = 1;
    DcMotor intakeMotor;

    private static final double BucketUpPosition = .92;
    private static final double BucketDownPosition = .46;
    Servo slantServo;
    public ColorRangeSensor intakeSensor;
    public DistanceSensor intakeSensor_D;


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
    public ContinuousIntake(HardwareMap hardwareMap, String motorName, String servoName, String colorSensor, boolean sensorDetectionLight) {
        intakeMotor = hardwareMap.dcMotor.get(motorName);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeSensor = hardwareMap.get(ColorRangeSensor.class, colorSensor);
        //intakeSensor_D = hardwareMap.get(DistanceSensor.class, colorSensor);
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
        double[] colorNumber = {intakeSensor.red(), intakeSensor.blue(), intakeSensor.green(), intakeSensor.alpha(), intakeSensor.argb()};
        switch (color) {
            case "red":
                i = 0;
                break;
            case "blue":
                i = 1;
                break;
            case "green":
                i = 2;
                break;
            case "alpha":
                i = 3;
                break;
            case "argb":
                i = 4;
                break;
            default:
                i = 0;
        }
        return colorNumber[i];

    }

    public double[] getRGB() {

        double[] colorNumber = {intakeSensor.red(), intakeSensor.green(), intakeSensor.blue()};
        return colorNumber;

    }

    public double getCloseDistance() {
        double distance = intakeSensor.getDistance(DistanceUnit.INCH);
        return distance;
    }

    public double linearEquation(double m, double b, double x) {
        double y = (m * x) + b;
        return y;
    }
    //ball red or green: y = 3.5x +43.833
    //ball blue: y = -3x +29
    //cube red or green: y = -4.5 + 45.167
    //cube blue: y= -1.75x + 20.417
    //duck green or red: y = -2x + 30.667
    //duck blue: y = 0.5x + 10.833
    // y is the color value for x(distance from sensor)


    public String getObjectType() {
        String s = null;
        double red = this.getColor("red");
        double green = this.getColor("green");
        double blue = this.getColor("blue");
        double distance = this.getCloseDistance();
        if (Math.abs(blue - linearEquation(-3, 29, distance)) >= Math.abs(blue - linearEquation(.5, 10.833, distance))) {
            s = "ball";
        }
        /*else if () {
            s = "duck";
        } else if () {
            s = "cube";
        }else{
            s= "none";
        }

         */
        return s;

    }


}
