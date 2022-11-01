package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.params.DriveParams;

public class ArmSystem {
    public enum ArmLevel {
        LOW,
        MEDIUM,
        HIGH
    }
    //fill in constants
    private final int POSITION_LOW_MM = 0;
    private final int POSITION_MEDIUM_MM = 0;
    private final int POSITION_HIGH_MM = 0;
    private final double MAX_POWER = 0;
    private final double  TICKS_IN_MM = ticksInMm();
    private final DcMotor armLeft;
    private final DcMotor armRight;


    public ArmSystem(DcMotor motor1, DcMotor motor2){
        armLeft = motor1;
        armRight = motor2;
        initMotors();
    }
    public void initMotors() {
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setPower(0);
    }
    public void driveToLevel(ArmLevel level){
        int targetPosition = millimetersToTicks(POSITION_LOW_MM);
        switch (level){
            case MEDIUM:
                targetPosition = millimetersToTicks(POSITION_MEDIUM_MM);
                break;
            case HIGH:
                targetPosition = millimetersToTicks(POSITION_HIGH_MM);
                break;
        }
        //add code for second motor (armRight)
        armLeft.setTargetPosition(targetPosition);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeft.setPower(MAX_POWER);
    }

    //Conversions
    private int millimetersToTicks(int millimeters) {
        return (int) Math.round(millimeters * TICKS_IN_MM);
    }
    private double ticksInMm() {
        return DriveParams.TICKS_PER_REV / inchesToMm(DriveParams.CIRCUMFERENCE);
    }
    private double inchesToMm(double inch) {
        return inch * 25.4;
    }


    public static class Intake {
        private final DigitalChannel beamBreaker;
        private final DcMotorSimple coneTake;
        private final ElapsedTime elapsedTime;
        public enum State { IDLE, INTAKING, OUTTAKING }
        private State state;

        public Intake(DcMotorSimple intake, DigitalChannel beam){
            beamBreaker = beam;
            beamBreaker.setMode(DigitalChannel.Mode.INPUT);
            coneTake = intake;
            state = State.IDLE;
            elapsedTime = new ElapsedTime();
        }

        public boolean isBeamBroken(){
            return !beamBreaker.getState();
        }

        public static boolean intake(){
            if (state != State.INTAKING) {
                state = State.INTAKING;
                coneTake.setDirection(DcMotor.Direction.REVERSE);
                coneTake.setPower(0.75);
            }

            if (isBeamBroken()) {
                coneTake.setPower(0.0);
                state = State.IDLE;
            }

            return state == State.IDLE;
        }

        public static boolean outtake(){
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