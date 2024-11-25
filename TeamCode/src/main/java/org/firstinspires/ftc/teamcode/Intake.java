package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class Intake {

    HashMap<String, Double> savedPositions = new HashMap<String, Double>();

    DcMotorEx horizontalSlide, intakeMotor;
    Servo gate;

    final double gateOpen = 0.5;
    final double gateClose = 0.1;


    //-14 0
    //171 3

    final double TICKS_PER_INCH = 61.666667;//175.5
    final double MIN_POINT = 6;
    final double MAX_POINT = 24;
    final double DEFAULT_ALLOWED_ERROR = 0.5;


    public Intake(HardwareMap hardwareMap) {
        horizontalSlide = hardwareMap.get(DcMotorEx.class, "slideHoriz");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");

        horizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        gate.setDirection(Servo.Direction.FORWARD);

        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        savedPositions.put("transfer", MIN_POINT);
        savedPositions.put("max", MAX_POINT);
    }

    public void transfer() {
        setHorizontalSlideToSavedPosition("transfer");
        if (isAtSavedPosition("transfer", 1.5)) {
            intakeMotor.setPower(0.6);
            openGate();
        } else {
            intakeMotor.setPower(1);
            closeGate();
        }
    }
    public void setHorizontalSlideToSavedPosition(String key) {
        setHorizontalSlidePositionInches(savedPositions.get(key));
    }

    public boolean isAtSavedPosition(String key) {
        return isAtPosition(savedPositions.get(key), DEFAULT_ALLOWED_ERROR);
    }

    public boolean isAtSavedPosition(String key, double allowedError) {
        return isAtPosition(savedPositions.get(key), allowedError);
    }
    public boolean isAtPosition(double inches) {
        return isAtPosition(inches, DEFAULT_ALLOWED_ERROR);
    }
    public boolean isAtPosition(double inches, double allowedError) {
        return RobotMath.isAbsDiffWithinRange(inches, currentInches(), allowedError);
    }

    public double currentInches() {
        return ticksToInches(horizontalSlide.getCurrentPosition());
    }

    public void setHorizontalSlidePositionInches(double inches) {
        double tickTarget = inchesToTicks(RobotMath.maxAndMin(inches, 24, 6));
        double error = tickTarget - horizontalSlide.getCurrentPosition();

        horizontalSlide.setPower(0.01 * error);
    }

    public double inchesToTicks(double inches) {
        return (inches - MIN_POINT) * TICKS_PER_INCH;
    }
    public double ticksToInches(double ticks) {
        return (ticks / TICKS_PER_INCH) + MIN_POINT;
    }


    public void setHorizontalSlidePower(double power) {
        horizontalSlide.setPower(power);
    }
    public void setIntakeMotorPower(double power) {
        intakeMotor.setPower(power);
    }

    public void resetEncoder() {
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //GATE
    public void openGate() {
        gate.setPosition(gateOpen);
    }
    public void closeGate() {
        gate.setPosition(gateClose);
    }
    public boolean isGateOpen() {
        return RobotMath.isAbsDiffWithinRange(gateOpen, gate.getPosition(), 0.001);
    }
}
