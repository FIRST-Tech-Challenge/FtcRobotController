package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Hardware;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeMotorStates.INTAKE_STILL;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeServoStates.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeMotorStates.INTAKE_REVERSING;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeServoStates.INTAKE_FLIPPING_DOWN;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeServoStates.INTAKE_FLIPPING_UP;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeServoStates.INTAKE_UP;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.RobotStates.INTAKE_SWITCHED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeMotorStates.INTAKING;

import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.checker;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.ColorDistanceRevV3;

public class Intake {

    private DcMotorEx intakeMotor = null;
    private Servo intakeServo = null;
    private Servo intakeServo2 = null;
    DigitalChannel touchSensor;
    private ColorDistanceRevV3 sensorDistance;
    boolean isIntaking = false;
    double intakeTimeStart = -10;
    double flipTime = 100;

    final private double intakeSpeed = 1;
    boolean teleop = false;
    private boolean intakeUp = false; //position of intake in the lifted position

    // initialization of intakeMotor
    public Intake(boolean isTeleop, StateMachine checkers) {
        checker = checkers;
        teleop = isTeleop;
        intakeMotor = (DcMotorEx) op.hardwareMap.dcMotor.get("IntakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo = op.hardwareMap.get(Servo.class, "IntakeServo");
        intakeServo2 = op.hardwareMap.get(Servo.class, "IntakeServo2");
//        touchSensor = opMode.hardwareMap.get(DigitalChannel.class, "touchSensor");
        sensorDistance = new ColorDistanceRevV3();
//        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        if (!isTeleop) {
            intakeServo.setPosition(.24);
            intakeServo2.setPosition(.76);
        }
        flipTime = 0;
    }
    public void updateIntakeStates(){
        isSwitched();
        if(op.getRuntime()-flipTime>0.7){
            if (checker.getState(INTAKE_FLIPPING_UP)) {
                checker.setState(INTAKE_UP, true);
            }
            else if (checker.getState(INTAKE_FLIPPING_DOWN)){
                checker.setState(INTAKE_DOWN, true);
            }
        }

        if (intakeMotor.getPower() > 0) {
            checker.setState(INTAKING, true);
        }
        else if (intakeMotor.getPower() == 0) {
            checker.setState(INTAKE_STILL, true);
        }
        else {
            checker.setState(INTAKE_REVERSING, true);
        }
    }

    public boolean isSwitched() {
        if (sensorDistance.getSensorDistance()<1.5) {
            checker.setState(INTAKE_SWITCHED, true);
            op.telemetry.addData("switch", sensorDistance.getSensorDistance());
        }
        return (sensorDistance.getSensorDistance()<1.5);
    }

    public void startIntake() {

            intakeTimeStart = op.getRuntime();
//            checker.setState(INTAKE_REVERSING, false);
            intakeMotor.setPower(intakeSpeed);
    }

    public void spinIntake (double power) {
        intakeMotor.setPower(power);
    }

    public void reverseIntake(double power) {
        checker.setState(INTAKE_REVERSING, true);
        intakeMotor.setPower(-power);
//        checker.setState(INTAKING, false);

    }

//    public void transferIntake(double power) {
//        intakeMotor.setPower(-power);
////        checker.setState(INTAKING, false);
//        checker.setState(INTAKE_TRANSFERRING, true);
//    }


    public void stopIntake() {
//        checker.setState(INTAKING, false);
//        checker.setState(INTAKE_REVERSING, false);
//        checker.setState(INTAKE_TRANSFERRING, false);
        intakeMotor.setPower(0);
        checker.setState(INTAKE_STILL, true);
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
        if (checker.checkIf(INTAKE_FLIPPING_UP) &&op.getRuntime()-flipTime>0.4) {
            flipTime= op.getRuntime();
            intakeServo.setPosition(.21);
            intakeServo2.setPosition(.79);
            checker.setState(INTAKE_FLIPPING_UP, true);
//            checker.setState(INTAKE_DOWN, false);
            return true;
        } else if(checker.checkIf(INTAKE_FLIPPING_DOWN)) {
            flipTime = op.getRuntime();
            intakeServo.setPosition(1.0);
            intakeServo2.setPosition(0.0);
            checker.setState(INTAKE_FLIPPING_DOWN, true);
//            checker.setState(INTAKE_DOWN, true);
            return true;
        }
        else{
            return false;
        }
    }

    public void flipIntakeToPosition(double torget) {
        if(op.getRuntime()-flipTime>0.4) {
            flipTime= op.getRuntime();
            if (torget > 0.5) {
                checker.setState(INTAKE_FLIPPING_UP, true);
            }
            else {
                checker.setState(INTAKE_FLIPPING_DOWN, true);

            }
            //            intakeServo.setPosition(1 - torget);
            intakeServo2.setPosition(0 + torget);
        }
    }

}