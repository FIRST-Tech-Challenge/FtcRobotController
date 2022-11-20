package org.firstinspires.ftc.teamcode.components;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.params.DriveParams;

public class ArmSystem {

    // Pole Heights
    public static final int LOW = 500;
    public static final int MEDIUM = 750;
    public static final int HIGH = 1100;
    public static final int FLOOR = 120;

    public enum Cone {
        ONE (100),
        TWO (120),
        THREE (150),
        FOUR (190),
        FIVE (320);

        private final int height;
        // Cone Stack Drop Distance
        public static final int CONE_DROP = 70;

        // Cone Clear Distance
        public static final int CONE_CLEAR = 100;

        Cone(int height) {
            this.height = height;
        }

        public int approach() {
            return height;
        }

        public int grab() {
            return height - CONE_DROP;
        }

        public int clear() {
            return height + CONE_CLEAR;
        }

    }

    // grab 2 approach 120
    // grab 2 grab 40
    // lift off 2 190

    // grab 3 approach 150 (120 + 30)
    // grab 3 grab 79 (40 + 40)
    // lift off 3 200

    // grab 4 approach 190 (150 + 40)
    // grab 4 grab 125 (80 + 40)
    // lift off 4 244

    // grab 5 approach 240 (190 + 50)
    // grab 5 170 (120 + 50)
    // lift off 5 270

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

    public void killMotors() {
        armLeft.setPower(0);
        armRight.setPower(0);
        mTargetPosition = 0;
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

    public boolean driveToLevel(int targetPosition, double power){
        if(mTargetPosition == 0){
            armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if(mTargetPosition != targetPosition){
            mTargetPosition = targetPosition;
            armLeft.setTargetPosition(targetPosition);
            armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLeft.setPower(power);
            armRight.setTargetPosition(targetPosition);
            armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRight.setPower(power);
        }

        //add code for second motor (armRight)
        int offsetLeft = Math.abs(armLeft.getCurrentPosition() - targetPosition);
        int offsetRight = Math.abs(armRight.getCurrentPosition() - targetPosition);
        Log.d("what is happening", offsetLeft + " " + offsetRight + " " + armLeft.getCurrentPosition() + " " + armRight.getCurrentPosition() + " power " + armLeft.getPower() + " " + armRight.getPower());
        if(targetPosition != 0 && offsetLeft < 20 && offsetRight < 20 ){
            Log.d("reached", armLeft.getCurrentPosition() + " " + armRight.getCurrentPosition() + " power " + armLeft.getPower() + " " + armRight.getPower() );
            return true;
        }
        else if (targetPosition > 0 && offsetLeft < 15 && offsetRight < 20){
            return true;
        }
        return false;
    }

    public void toDaGround(){
        if(driveToLevel(FLOOR, 0.2)){
            armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armRight.setPower(0);
            armLeft.setPower(0);
        }
    }

    public boolean up() {
        return (armRight.getCurrentPosition() + armLeft.getCurrentPosition())/2 > 400;
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

        public boolean isBeamBroken(){
            return !beamBreaker.getState();
        }

        public boolean intake(){


            if (isBeamBroken()) {
                if(intakeDelay < 0 ){
                    elapsedTime.reset();
                    intakeDelay = elapsedTime.milliseconds();
                }
                if(elapsedTime.milliseconds() - intakeDelay > 100){
                    coneTake.setPower(0.0);
                    state = State.IDLE;
                }
            } else if (state != State.INTAKING) {
                state = State.INTAKING;
                coneTake.setDirection(DcMotor.Direction.REVERSE);
                coneTake.setPower(0.75);
            }
            return state == State.IDLE;
        }

        public boolean outtake(){
            if (state != State.OUTTAKING) {
                state = State.OUTTAKING;
                coneTake.setDirection(DcMotorSimple.Direction.FORWARD);
                coneTake.setPower(0.75);
                elapsedTime.reset();
            }

            if (elapsedTime.milliseconds() > 100) {
                state = State.IDLE;
                coneTake.setPower(0);
            }

            return state == State.IDLE;
        }

        public State getState() {
            return state;
        }

    }
}