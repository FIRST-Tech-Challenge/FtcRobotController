package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Worm extends SubsystemBase {

    private Telemetry telemetry;
    private DcMotorEx motor;
    private boolean raising;
    private int raisingPosition;
    private boolean lowering;
    private int loweringPosition;

    private static final double DEGREE_TO_TICK_MULTIPLIER = 537.6 / 360;

    public Worm(HardwareMap hm, Telemetry tm){

        motor = hm.get(DcMotorEx.class, "Tilt");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry = tm;
    }


    public void raise() {
        raise(0.25);
    }

    public void raise(double whatPower) {
        telemetry.addData("WormState", "raise");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        raising = true;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(whatPower);
    }

    public void lower() {
        lower(-0.25);
    }

    public void lower(double whatPower) {
        telemetry.addData("WormState", "lower");
        lowering = true;
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(whatPower);
    }

//    public void lower(int position) {
//        loweringPosition = position;
//        lower(-0.5);
//    }

//    public void setPower(double whatPower) {
//        if (whatPower<0) {
//            lower(whatPower);
//        } else if (whatPower>0) {
//            raise(whatPower);
//        } else {
//            brake();
//        }
//    }

    public void brake(){
        telemetry.addData("WormState", "stop");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }

    public double getDistance(){
        return motor.getCurrentPosition();
    }

    //TODO: fix these later
//    public boolean isLowered(){
//        return getDistance() <= 0 || getDistance() == loweringPosition;
//    }

//    public boolean isRaised(){
//        return getDistance() >= 3300 || getDistance() == raisingPosition;
//    }

    @Override
    //this function fires every cycle, at about 50hz, so anything in here will effectively be the default state
    public void periodic() {
        //telemetry.addData("WormIsLowered", isLowered());
        //telemetry.addData("WormIsRaised", isRaised());
        telemetry.addData("WormDistance", getDistance());

//        if (isRaised() && raising) {
//            telemetry.addData("WormIsRaisedAndTryingTorRaise", "true");
//            raising = false;
//            raisingPosition = Integer.MIN_VALUE;
//            brake();
//        }
    }
}