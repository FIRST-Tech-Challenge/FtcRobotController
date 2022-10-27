package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSystem {

    private int currentPos;
    private final DcMotor armLeft;
    private final DcMotor armRight;

    public ArmSystem(DcMotor motor1, DcMotor motor2){
        armLeft = motor1;
        armRight = motor2;

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