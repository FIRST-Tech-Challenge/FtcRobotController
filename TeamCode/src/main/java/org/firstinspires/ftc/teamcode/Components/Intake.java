package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {

    private DcMotorEx intakeMotor = null;
    private Servo intakeServo = null;
    private Servo intakeServo2 = null;
    DigitalChannel touchSensor;
    boolean isIntaking = false;
    double intakeTimeStart = -10;
    StateMachine checker = null;
    double flipTime = 0;

    LinearOpMode op = null;
    final private double intakeSpeed = 1;
    private DistanceSensor sensorDistance;
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
        sensorDistance = opMode.hardwareMap.get(DistanceSensor.class, "intakeDistSensor");
//        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        if (!isTeleop) {
            intakeServo.setPosition(.31);
            intakeServo2.setPosition(.69);
        }
        flipTime = 0;
    }
    public void updateIntakeStates(){
        isSwitched();
        if(intakeServo.getPosition()<0.5){
            checker.setState(StateMachine.States.INTAKE_DOWN, false);
        }
        else{
            checker.setState(StateMachine.States.INTAKE_DOWN, true);

        }
        if(op.getRuntime()-flipTime>0.7){
            checker.setState(StateMachine.States.FLIPPING, false);
        }
    }

    public boolean isSwitched() {
            checker.setState(StateMachine.States.SWITCHED, sensorDistance.getDistance(DistanceUnit.INCH)<1);
            op.telemetry.addData("switch", sensorDistance.getDistance(DistanceUnit.INCH));
            return (sensorDistance.getDistance(DistanceUnit.INCH)<1);
    }

    public void startIntake() {

            intakeTimeStart = op.getRuntime();
            checker.setState(StateMachine.States.INTAKING, true);
            checker.setState(StateMachine.States.REVERSING, false);
            intakeMotor.setPower(intakeSpeed);
    }

    public void reverseIntake(double power) {
        intakeMotor.setPower(-power);
        checker.setState(StateMachine.States.INTAKING, false);
        checker.setState(StateMachine.States.REVERSING, true);
    }


    public void stopIntake() {
        checker.setState(StateMachine.States.INTAKING, false);
        checker.setState(StateMachine.States.REVERSING, false);
        checker.setState(StateMachine.States.TRANSFERRING, false);
        intakeMotor.setPower(0);
    }


//    public void stopperUp() {
//        stopperServo.setPosition(stopperUp);
//    }
//
//    public void stopperDown() {
//        stopperServo.setPosition(0);
//    }

    public boolean flipIntake() {
        if (checker.getState(StateMachine.States.INTAKE_DOWN)&&checker.checkIf(StateMachine.States.FLIPPING) &&op.getRuntime()-flipTime>0.4) {
            flipTime= op.getRuntime();
            intakeServo.setPosition(.28);
            intakeServo2.setPosition(.72);
            checker.setState(StateMachine.States.FLIPPING, true);
            checker.setState(StateMachine.States.INTAKE_DOWN, false);
            return true;
        } else if(!checker.getState(StateMachine.States.FLIPPING)) {
            flipTime = op.getRuntime();
            intakeServo.setPosition(1.0);
            intakeServo2.setPosition(0.0);
            checker.setState(StateMachine.States.FLIPPING, true);
            checker.setState(StateMachine.States.INTAKE_DOWN, true);
            return true;
        }
        else{
            return false;
        }
    }

    public void flipIntakeToPosition(double torget) {
        if(op.getRuntime()-flipTime>0.4) {
            flipTime= op.getRuntime();
            checker.setState(StateMachine.States.FLIPPING, true);
            intakeServo.setPosition(1 - torget);
            intakeServo2.setPosition(0 + torget);
        }
    }

}