package org.firstinspires.ftc.teamcode.Components;
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
    boolean isIntaking = false;
    double intakeTimeStart = -10;
    StateMachine checker = null;

    LinearOpMode op = null;
    final private double intakeSpeed = 1;
    boolean teleop = false;
    private boolean intakeUp = false; //position of intake in the lifted position

    // initialization of intakeMotor
    public Intake(LinearOpMode opMode, boolean isTeleop, StateMachine checkers){
        checker = checkers;
        op = opMode;
        teleop = isTeleop;
        intakeMotor = (DcMotorEx) opMode.hardwareMap.dcMotor.get("IntakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo = opMode.hardwareMap.get(Servo.class, "IntakeServo");
        intakeServo2 = opMode.hardwareMap.get(Servo.class, "IntakeServo2");
        touchSensor = opMode.hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        if(!isTeleop) {
            intakeServo.setPosition(.3);
            intakeServo2.setPosition(1.0);
        }
    }

    public boolean isSwitched() {
        if(!teleop) {
            if (intakeMotor.getVelocity() < 2000 && isIntaking && op.getRuntime() - intakeTimeStart > 0.8 || !touchSensor.getState()) {
                return false;
            }
        }
        else{
            checker.setState(StateMachine.States.SWITCHED, !touchSensor.getState());
            return touchSensor.getState();
        }
        return true;
    }

    public void startIntake() {
        if(!isIntaking){
            intakeTimeStart = op.getRuntime();
        }
        isIntaking = true;
        intakeMotor.setPower(intakeSpeed);
    }

    public void reverseIntake(double power){
        isIntaking = false;
        intakeMotor.setPower(-power);
    }


    public void stopIntake() {
        isIntaking = false;
        intakeMotor.setPower(0);
    }


//    public void stopperUp() {
//        stopperServo.setPosition(stopperUp);
//    }
//
//    public void stopperDown() {
//        stopperServo.setPosition(0);
//    }

    public boolean flipIntake(){
        if (checker.checkIf(StateMachine.States.FLIPPING)) {
            if (intakeServo.getPosition()<0.5) {
                intakeServo.setPosition(1);
                intakeServo2.setPosition(0.3);
                intakeUp = true;
            } else {
                intakeServo.setPosition(.3);
                intakeServo2.setPosition(1.0);
                intakeUp = false;
            }
            return true;
        }
        else {
            return false;
        }
    }
    public void flipIntakeToPosition(double torget){
            intakeServo.setPosition(1 - torget);
            intakeServo2.setPosition(.3 + torget);
    }

}