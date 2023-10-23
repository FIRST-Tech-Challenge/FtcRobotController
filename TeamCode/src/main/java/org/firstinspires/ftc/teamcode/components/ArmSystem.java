package org.firstinspires.ftc.teamcode.components;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.params.DriveParams;

public class ArmSystem {

    public enum Backdrop {

    }

    public DcMotor armLeft; //arm left is motor1
    public DcMotor armRight;
    public Intake intake;
    private int mTargetPosition;


    public ArmSystem(DcMotor motor1, DcMotor motor2, DcMotor intakeMotor, DigitalChannel beam){
        armLeft = motor1;
        armRight = motor2;
        initMotors();
        intake = new Intake(intakeMotor, beam);
        mTargetPosition = 0;
    }

    public boolean isBusy() {
        return armLeft.isBusy() || armRight.isBusy();
    }

    public static void armToFloor() {

    }

    public Intake.State getState(){
        return intake.getState();
    }

    public boolean intake(){
        return intake.intake();
    }

    public boolean outtake(){
        return intake.outtake();
    }

    public void initMotors() {
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setPower(0);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setPower(0);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static class Intake {
        private final DigitalChannel beamBreaker;
        private final DcMotorSimple coneTake;
        private final ElapsedTime elapsedTime;
        public enum State { IDLE, INTAKING, OUTTAKING }
        private State state;
        private double intakeDelay;

        public Intake(DcMotorSimple intake, DigitalChannel beam){
            beamBreaker = beam;
            beamBreaker.setMode(DigitalChannel.Mode.INPUT);
            coneTake = intake;
            state = State.IDLE;
            elapsedTime = new ElapsedTime();
            intakeDelay = -1.0;
        }


        public boolean intake() {
            return true;
        }

        public boolean outtake(){
           return true;
        }

        public State getState() {
            return state;
        }

    }
}