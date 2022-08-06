package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKE_REVERSING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKE_SWITCHED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKE_TRANSFERRING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKING;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotorEx intakeMotor = null;
    private Servo intakeServo = null;
    private Servo intakeServo2 = null;
    DigitalChannel touchSensor;
    private ColorDistanceRevV3 sensorDistance;
    boolean isIntaking = false;
    double intakeTimeStart = -10;
    StateMachine checker = null;
    double flipTime = 0;

    LinearOpMode op = null;
    final private double intakeSpeed = 1;
    boolean teleop = false;
    private boolean intakeUp = false; //position of intake in the lifted position

    // initialization of intakeMotor
    public Intake(LinearOpMode opMode, boolean isTeleop, StateMachine checkers) {
        checker = checkers;
        op = opMode;
        teleop = isTeleop;
        intakeMotor = (DcMotorEx) opMode.hardwareMap.dcMotor.get("IntakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo = opMode.hardwareMap.get(Servo.class, "IntakeServo");
        intakeServo2 = opMode.hardwareMap.get(Servo.class, "IntakeServo2");
//        touchSensor = opMode.hardwareMap.get(DigitalChannel.class, "touchSensor");
        sensorDistance = new ColorDistanceRevV3(op);
//        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        if (!isTeleop) {
            intakeServo.setPosition(.24);
            intakeServo2.setPosition(.76);
        }
        flipTime = 0;
    }
    public void updateIntakeStates(){
        isSwitched();
        if(intakeServo.getPosition()<0.5){
            checker.setState(INTAKE_DOWN, false);
        }
        else{
            checker.setState(INTAKE_DOWN, true);

        }
        if(op.getRuntime()-flipTime>0.7){
            checker.setState(StateMachine.IntakeStates.INTAKE_FLIPPING_UP, false);
        }
    }

    public boolean isSwitched() {
            checker.setState(INTAKE_SWITCHED, sensorDistance.getSensorDistance()<1.5);
            op.telemetry.addData("switch", sensorDistance.getSensorDistance());
            return (sensorDistance.getSensorDistance()<1.5);
    }

    public void startIntake() {

            intakeTimeStart = op.getRuntime();
            checker.setState(INTAKING, true);
            checker.setState(INTAKE_REVERSING, false);
            intakeMotor.setPower(intakeSpeed);
    }

    public void reverseIntake(double power) {
        intakeMotor.setPower(-power);
        checker.setState(INTAKING, false);
        checker.setState(INTAKE_REVERSING, true);
    }


    public void stopIntake() {
        checker.setState(INTAKING, false);
        checker.setState(INTAKE_REVERSING, false);
        checker.setState(INTAKE_TRANSFERRING, false);
        intakeMotor.setPower(0);
    }


//    public void stopperUp() {
//        stopperServo.setPosition(stopperUp);
//    }
//
//    public void stopperDown() {
//        stopperServo.setPosition(0);
//    }
    public boolean isBall(){
        return sensorDistance.isBall();
    }
    public boolean flipIntake() {
        if (checker.getState(INTAKE_DOWN)&&checker.checkIf(StateMachine.IntakeStates.INTAKE_FLIPPING_UP) &&op.getRuntime()-flipTime>0.4) {
            flipTime= op.getRuntime();
            intakeServo.setPosition(.21);
            intakeServo2.setPosition(.79);
            checker.setState(StateMachine.IntakeStates.INTAKE_FLIPPING_UP, true);
            checker.setState(INTAKE_DOWN, false);
            return true;
        } else if(!checker.getState(StateMachine.IntakeStates.INTAKE_FLIPPING_UP)) {
            flipTime = op.getRuntime();
            intakeServo.setPosition(1.0);
            intakeServo2.setPosition(0.0);
            checker.setState(StateMachine.IntakeStates.INTAKE_FLIPPING_DOWN, true);
            checker.setState(INTAKE_DOWN, true);
            return true;
        }
        else{
            return false;
        }
    }

    public void flipIntakeToPosition(double torget) {
        if(op.getRuntime()-flipTime>0.4) {
            flipTime= op.getRuntime();
            if (torget > intakeServo2.getPosition()) {
                checker.setState(StateMachine.IntakeStates.INTAKE_FLIPPING_UP, true);
            }
            else {
                checker.setState(StateMachine.IntakeStates.INTAKE_FLIPPING_DOWN, true);

            }
            intakeServo.setPosition(1 - torget);
            intakeServo2.setPosition(0 + torget);
        }
    }

}