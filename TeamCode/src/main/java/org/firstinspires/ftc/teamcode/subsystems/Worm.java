package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Worm extends SubsystemBase {

    private final PIDFController controller;
    private Telemetry telemetry;
    private DcMotorEx motor;
    private double potOffset = 154.4615;
    public static PIDFCoefficients WORM_PID = new PIDFCoefficients(0.07, 0, 0,0);
    private final InterpLUT angleLookup = new InterpLUT();
    private final AnalogInput pot;

    private enum State {
        Raising,
        Lowering,
        Stopped
    }
    private State CurrentState;

    public Worm(HardwareMap hm, Telemetry tm){
        pot = hm.get(AnalogInput.class, "wormpot");
        motor = hm.get(DcMotorEx.class, "Tilt");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = tm;

        angleLookup.add(-1,0);
        angleLookup.add(0.186, 110);
        angleLookup.add(0.284, 100);
        angleLookup.add(0.373, 90);
        angleLookup.add(0.454, 80);
        angleLookup.add(0.537, 70);
        angleLookup.add(0.61, 60);
        angleLookup.add(0.687, 50);
        angleLookup.add(0.76, 40);
        angleLookup.add(0.835, 30);
        angleLookup.add(0.92, 20);
        angleLookup.add(0.976, 10);
        angleLookup.add(1.051, 0);
        angleLookup.add(1.09, -5);
        angleLookup.add(1.168, -15);
        angleLookup.add(1.239, -25);

        angleLookup.createLUT();

        controller = new PIDFController(WORM_PID.p, WORM_PID.i, WORM_PID.d, 0);
        CurrentState = State.Stopped;
    }


    public double getAngle() {
        final double potentiometerAngle = angleLookup.get(pot.getVoltage());
        return (potentiometerAngle - potOffset);
    }

    public void raise() {
        raise(0.25);
    }

    public void raise(double whatPower) {
        telemetry.addData("WormState", "raise");
        CurrentState = State.Raising;
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setPower(whatPower);
    }

    public void lower() {
        lower(-0.25);
    }

    public void lower(double whatPower) {
        telemetry.addData("WormState", "lower");
        CurrentState = State.Lowering;
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setPower(whatPower);
    }

    public void brake(){
        CurrentState = State.Stopped;
        motor.setPower(0);
    }

    public void setPower(double whatPower) {
        telemetry.addData("Worm Power", whatPower);
        if (whatPower > 0) {
            raise(whatPower);
        } else {
            lower(whatPower);
        }
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
        telemetry.addData("WormAngle", getAngle());
        telemetry.addData("WormState", CurrentState.toString());
        telemetry.addData("WormPotVoltage", pot.getVoltage());

//        if (isRaised() && raising) {
//            telemetry.addData("WormIsRaisedAndTryingTorRaise", "true");
//            raising = false;
//            raisingPosition = Integer.MIN_VALUE;
//            brake();
//        }
    }
}