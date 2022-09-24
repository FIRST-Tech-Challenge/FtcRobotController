package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class IntakeBot extends GyroBot{
    public DcMotor intakeMotor = null;
    public Servo intakeArm = null;
    public Servo intakeHeight1 = null;
    public Servo intakeHeight2 = null;

    final double feederDown = 0.195;//0.21
    final double feederShake = 0.29;
    final double feederUp = 0.47;

    final double heightHigh = 0.275;//0.265
    final double heightLow = 0.303;

    public boolean isDown = true;
    public boolean intakeOn = false;
    public boolean isForward = true;

    long lastToggleDone2 = 0;
    long timeSinceToggle2 = 0;
    long lastToggleDone3 = 0;
    long timeSinceToggle3 = 0;

    public IntakeBot(LinearOpMode opMode) {
        super(opMode);
    }

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        intakeMotor = hwMap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeArm = hwMap.get(Servo.class, "intakeArm");
        intakeArm.setPosition(feederDown);
        intakeHeight1 = hwMap.get(Servo.class, "intakeHeight1");
        intakeHeight1.setPosition(heightHigh);
        intakeHeight2 = hwMap.get(Servo.class, "intakeHeight2");
        intakeHeight2.setPosition(heightHigh);
    }

    public void startIntake() {
        intakeMotor.setPower(0.8);
    }
    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public void toggleFeeder(boolean button) {
        timeSinceToggle2 = System.currentTimeMillis() - lastToggleDone2;
        if (button && timeSinceToggle2 > 290) {
            if (isDown) {
                intakeArm.setPosition(feederUp);
                isDown = false;
                lastToggleDone2 = System.currentTimeMillis();
            } else if (!isDown) {
                intakeArm.setPosition(feederDown);
                isDown = true;
                lastToggleDone2 = System.currentTimeMillis();
            }
        }
//        opMode.telemetry.addData("lastToggle", timeSinceToggle);
//        opMode.telemetry.update();
    }

    public void intakeControl(boolean button) {
        timeSinceToggle3 = System.currentTimeMillis() - lastToggleDone3;
        if (button && timeSinceToggle3 > 400) {
            if (intakeOn) {
                stopIntake();
                raiseIntake();
                intakeOn = false;
                lastToggleDone3 = System.currentTimeMillis();
            } else if (!intakeOn && isDown) {
                startIntake();
                lowerIntake();
                intakeOn = true;
                lastToggleDone3 = System.currentTimeMillis();
            }
        }
    }

    public void directionToForward(boolean button) {
        if (button) {
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            isForward = true;
        }
    }
    public void directionToReverse(boolean button) {
        if (button) {
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            isForward = false;
        }
    }

    public void shakeFeeder(boolean button) {
        intakeArm.setPosition(feederDown);
        if (button) {
            for (int i = 0; i < 10; i++) {
                intakeArm.setPosition(feederShake);
                sleep(200, "feeder shake");
                intakeArm.setPosition(feederDown);
                sleep(200, "feeder down");
            }
        }
    }

    public void raiseIntake() {
        intakeHeight1.setPosition(heightHigh);
        intakeHeight2.setPosition(heightHigh);
    }
    public void lowerIntake() {
        intakeHeight1.setPosition(heightLow);
        intakeHeight2.setPosition(heightLow);
    }

}
