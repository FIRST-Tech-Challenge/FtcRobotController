package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class WobbleGoalBot extends NoodleSpinnerBot {
    public Servo wobblePinch = null;
    public DcMotor wobbleArm = null;

    //two positions of the wobble servo
    final double wobblePinched = 0.7;
    final double wobbleOpened = 0.8;

    final int[] armPositions = new int[]{-25, -580, -760, -910};
    int armPosIndex = 0;
    public final double[] servoPositions = new double[]{0.19, 0.32, 0.4, 0.35};
    public int servoPosIndex = 1;

    public boolean isOpen = true;
    long lastToggleDone = 0;
    long timeSinceToggle = 0;
    long lastPosSwitch = 0;
    long timeSincePosSwitch = 0;
    long lastPosSwitch1 = 0;
    long timeSincePosSwitch1 = 0;

    public WobbleGoalBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        wobblePinch = hwMap.get(Servo.class, "wobblePinch");
        wobblePinch.setPosition(servoPositions[servoPosIndex]);
        wobbleArm = hwMap.get(DcMotor.class, "wobbleArm");
        wobbleArm.setPower(0);
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //call openPinch() to open the arm
    public void openPinch() {
        wobblePinch.setPosition(wobblePinched);
    }

    //call closeArm() to close the arm
    public void closePinch() {
        wobblePinch.setPosition(wobbleOpened);
    }

    public void toggleWobble(boolean button) {
        timeSinceToggle = System.currentTimeMillis() - lastToggleDone;
        if (button && timeSinceToggle > 300) {
            if (isOpen) {
                wobblePinch.setPosition(wobblePinched);
                isOpen = false;
                lastToggleDone = System.currentTimeMillis();
            } else if (!isOpen) {
                wobblePinch.setPosition(wobbleOpened);
                isOpen = true;
                lastToggleDone = System.currentTimeMillis();
            }
        }
//        opMode.telemetry.addData("lastToggle", timeSinceToggle);
//        opMode.telemetry.update();
    }

    public void raiseArm() {
        wobbleArm.setPower(0.3);
        wobbleArm.setTargetPosition(500);
        sleep(1000, "wobble raise");
        wobbleArm.setPower(0);
    }
    public void lowerArm() {
        wobbleArm.setPower(0.1);
        wobbleArm.setTargetPosition(200);
        sleep(500, "wobble lower");
        wobbleArm.setPower(0);
    }

    public void controlWobbleArm(boolean buttonY, boolean buttonB) {
        timeSincePosSwitch = System.currentTimeMillis() - lastPosSwitch;
        if (buttonY && timeSincePosSwitch > 350) {
            if (armPosIndex < 3 && armPosIndex > 0) {
                armPosIndex ++;
                wobbleArm.setTargetPosition(armPositions[armPosIndex]);
                wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArm.setPower(0.3);
                lastPosSwitch = System.currentTimeMillis();
            }
            if (armPosIndex == 0) {
                inOutPosIndex = 1;
                inOut.setPosition(inOutPositions[inOutPosIndex]);
                armPosIndex ++;
                wobbleArm.setTargetPosition(armPositions[armPosIndex]);
                wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArm.setPower(0.3);
                lastPosSwitch = System.currentTimeMillis();
            }
        }
        if (buttonB && timeSincePosSwitch > 350) {
            if (armPosIndex > 1) {
                armPosIndex --;
                wobbleArm.setTargetPosition(armPositions[armPosIndex]);
                wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArm.setPower(0.1);
                lastPosSwitch = System.currentTimeMillis();
            }
            if (armPosIndex == 1) {
                inOutPosIndex = 1;
                inOut.setPosition(inOutPositions[inOutPosIndex]);
                armPosIndex --;
                wobbleArm.setTargetPosition(armPositions[armPosIndex]);
                wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArm.setPower(0.1);
                servoPosIndex = 2;
                wobblePinch.setPosition(servoPositions[servoPosIndex]);
                lastPosSwitch = System.currentTimeMillis();
            }
        }
//        if (timeSincePosSwitch > 100 && lastPosSwitch != 0) {
//            wobbleArm.setPower(0.4);
//        }
        //opMode.telemetry.addData("armPosIndex", armPosIndex);
        //opMode.telemetry.update();
    }

    public void fineTuneWobbleArm(float down, float up) {
        if (up > 0){
            wobbleArm.setTargetPosition((int) (wobbleArm.getCurrentPosition()+up*25));
            wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleArm.setPower(0.1);
        } else if (down > 0){
            wobbleArm.setTargetPosition((int) (wobbleArm.getCurrentPosition()-down*25));
            wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleArm.setPower(0.1);
        }
    }

    public void controlServo(boolean dpadUp, boolean dpadDown) {
        timeSincePosSwitch1 = System.currentTimeMillis() - lastPosSwitch1;
        if (dpadUp && timeSincePosSwitch1 > 200) {
            if (servoPosIndex < 2) {
                servoPosIndex ++;
                wobblePinch.setPosition(servoPositions[servoPosIndex]);
                lastPosSwitch1 = System.currentTimeMillis();
            }
        }
        if (dpadDown && timeSincePosSwitch1 > 200) {
            if (servoPosIndex > 0) {
                servoPosIndex --;
                wobblePinch.setPosition(servoPositions[servoPosIndex]);
                lastPosSwitch1 = System.currentTimeMillis();
            }
        }
//        opMode.telemetry.addData("servoPosIndex", servoPosIndex);
//        opMode.telemetry.update();
    }

    public void setArmPosition(int position) {
        wobbleArm.setPower(0.2);
        wobbleArm.setTargetPosition(position);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (this.opMode.opModeIsActive() && wobbleArm.isBusy()) {
            sleep(50, "set wobble arm position");
        }
        wobbleArm.setPower(0.5);
    }

    public void setArmPositionNoWait(int position, double power) {
        wobbleArm.setPower(power);
        wobbleArm.setTargetPosition(position);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
