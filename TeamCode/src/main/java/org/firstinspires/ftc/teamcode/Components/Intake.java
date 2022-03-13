package org.firstinspires.ftc.teamcode.Components;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotor intakeMotor = null;
    private Servo intakeServo = null;
    private Servo intakeServo2 = null;
    DigitalChannel touchSensor;

    LinearOpMode op = null;
    final private double intakeSpeed = 1;
    private boolean intakeUp = false; //position of intake in the lifted position

    // initialization of intakeMotor
    public Intake(LinearOpMode opMode, boolean isTeleop){
        op=opMode;
        intakeMotor = opMode.hardwareMap.dcMotor.get("IntakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo = opMode.hardwareMap.get(Servo.class, "IntakeServo");
        intakeServo2 = opMode.hardwareMap.get(Servo.class, "IntakeServo2");
        touchSensor = opMode.hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        if(isTeleop){
            intakeServo.setPosition(1);
            intakeServo2.setPosition(.3);
        }
        else{
            intakeServo.setPosition(.35);
            intakeServo2.setPosition(.95);
        }
    }

    public boolean isSwitched() {
        return touchSensor.getState();
    }

    public void startIntake() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(intakeSpeed);
    }

    public void reverseIntake(double power){
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(-power);
    }


    public void stopIntake() {
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
        if (Turret.turretDown && Turret.turretStraight && Turret.basketDown) {
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
        else{
            return false;
        }
    }
    public void flipIntakeToPosition(double torget){
        if (Turret.turretDown && Turret.turretStraight && Turret.basketDown) {
            intakeServo.setPosition(1-torget);
            intakeServo2.setPosition(.3+torget);
        }
    }

}