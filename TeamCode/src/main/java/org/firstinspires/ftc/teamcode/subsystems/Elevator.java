package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.apache.commons.math3.analysis.function.Acos;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

public class Elevator extends SubsystemBase {
    private Telemetry telemetry;
    private DcMotorEx motor;
    private TouchSensor bottomLimit;
    private boolean isHomed;
    private double wormAngle;
    public ElevatorStatus Status;

    public enum ElevatorStatus
    {
        Extending,
        Retracting,
        Stopped
    }

    public Elevator(HardwareMap hm, Telemetry tm){
        wormAngle = 0; //the value of the worm angle so we can calculate max distance
        motor = hm.get(DcMotorEx.class, "Extend");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLimit = hm.get(TouchSensor.class, "Touch");
        telemetry = tm;
        this.Status = ElevatorStatus.Stopped;
    }

    public void SetWormAngle(double angle) {
        wormAngle = angle;
    }

    public void extend(){
        extend(0.5);
    }


    public void extend(double whatPower) {
        telemetry.addData("ElevatorState", "extend");
        if (isExtended()){
            brake();
        } else {
            this.Status = ElevatorStatus.Extending;
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(whatPower);
        }
    }

    public void retractTwoInches() {
        double currentDistance = this.getHorizontalExtension();
        double targetDistance = currentDistance - 2;
        while (targetDistance > currentDistance) {
            this.retract(0.5);
        }
        this.brake();
    }

//    public void extend(int position) {
//        extendingPosition = position;
//        extend(0.5);
//    }

    public void retract(){
        retract(-0.5);
    }

    public void retract(double whatPower) {
        telemetry.addData("ElevatorState", "extend");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(whatPower);

        if (isRetracted()){
            brake();
        } else {
            this.Status = ElevatorStatus.Retracting;
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(whatPower);
        }
    }

//    public void setPower(double whatPower) {
//        if (whatPower<0) {
//            retract(whatPower);
//        } else if (whatPower>0) {
//            extend(whatPower);
//        } else {
//            brake();
//        }
//    }

    public void brake(){
        telemetry.addData("ElevatorState", "stop");
        motor.setPower(0);
        this.Status = ElevatorStatus.Stopped;
    }

    public double getDistance(){
        return motor.getCurrentPosition();
    }

    public double getDistanceInInches() {
        return getDistance() * 0.01229202524;
    }

    public boolean isRetracted() {
        return bottomLimit.isPressed();
    }

    //this method returns true if we are maxed out on distance
    public boolean isExtended(){
        //TODO: calculate maximum distance based on angle

        //wormAngle
         // Changes angle to radians as java uses them by default
        double horizontalExtension = getHorizontalExtension();
        return horizontalExtension >= 21;
    }

    public double getHorizontalExtension() {
        double elevatorDistance = getDistanceInInches();
        return (elevatorDistance * Math.abs(Math.cos(Math.toRadians(wormAngle))));
    }

    //this function fires every cycle, at about 50hz, so anything in here will effectively be the default state
    @Override
    public void periodic() {
        telemetry.addData("ElevatorIsRetracted", isRetracted());
        telemetry.addData("ElevatorIsExtended", isExtended());
        telemetry.addData("ElevatorDistance", getDistance());
        telemetry.addData("ElevatorDistanceInInches", getDistanceInInches());
        telemetry.addData("ElevatorHorizontal", getHorizontalExtension());
        telemetry.addData("IsHomed", isHomed);
        telemetry.addData("ElevatorStatus", this.Status);

//        if (bottomLimit.isPressed() && !extending && !retracting) {
//            isHomed = true;
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//
//        if (bottomLimit.isPressed() && retracting) {
//            retracting = false;
//            retractingPosition = Integer.MIN_VALUE;
//            brake();
//        }
//
//        if (isExtended() && extending) {
//            extending = false;
//            extendingPosition = Integer.MIN_VALUE;
//            brake();
//        }
    }
}