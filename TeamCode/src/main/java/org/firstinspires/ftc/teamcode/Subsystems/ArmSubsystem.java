package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Range;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.MiniPID;

public class ArmSubsystem {
    //Variable Declarations
    private DcMotor armMotor;
    private double armMotorPower;
    private int armZeroPosition;

    private TouchSensor limitSwitch;

    MiniPID armMotorPID;

    //Constructor Class
    public ArmSubsystem(DcMotor armMotor, TouchSensor limitSwitch) {
        this.armMotor = armMotor;
        this.limitSwitch = limitSwitch;
        initialize();
    }

    //Initializes arm motor and PID controller. Finds starting position.
    public void initialize() {
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotorPID = new MiniPID(Constants.armP, Constants.armI, Constants.armD);
        armMotorPID.setOutputLimits(-1.0, 1.0);
    }

    private void autoPositionArm(Position position) {
        int targetPosition;
        switch (position) {
            case UP:
                targetPosition = Constants.armMotorMaxPosition;
                break;

            case HALFWAY:
                targetPosition = Constants.armMotorHalfwayPosition;
                break;

            case DOWN:
                targetPosition = armZeroPosition;
                break;

            default:
                return;
        }

        boolean inRange = false;
        while (!inRange) {
            armMotorPower = armMotorPID.getOutput(armMotor.getCurrentPosition(), targetPosition);
            if (-0.05 < armMotorPower && armMotorPower < 0.05) {
                inRange = true;
            }
            armMotor.setPower(armMotorPower);
        }
        armMotor.setPower(0.0);
    }

    private void zeroPosition() {
        while (!limitSwitch.isPressed()) {
            armMotor.setPower(Constants.armZeroingPower);
        }
        armMotor.setPower(0.0);
        armZeroPosition = (int) (armMotor.getCurrentPosition() + 56);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(armZeroPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }




    //Rotates the arm based off DPad input.
    public void ManualPositionArm(boolean gamepad2_DPadLeft, boolean gamepad2_DPadRight) {
        Range<Integer> rangeOfMotion = Range.create(armZeroPosition, Constants.armMotorMaxPosition);
        if (rangeOfMotion.contains(armMotor.getCurrentPosition())) {
            if (gamepad2_DPadRight) {
                armMotorPower = Constants.teleOPArmPower;
            }

            else if (gamepad2_DPadLeft) {
                armMotorPower = -Constants.teleOPArmPower;
            }

            else {
                armMotorPower = 0.0;
            }
        }

        //Moves the arm up or down to correct position if arm moves out of range.
        else if (armMotor.getCurrentPosition() < armZeroPosition) {
            armMotorPower = Constants.teleOPArmPower;
        }

        else {
            armMotorPower = -Constants.teleOPArmPower;
        }
    }

    //Moves the arm to the desired angle utilizing the arm PID.
    public void autoCustomPositionArm (double desiredAngle) {
        double targetPosition = (desiredAngle / 360) * Constants.armMotorCPR;
        com.qualcomm.robotcore.util.Range.clip(targetPosition, armZeroPosition, Constants.armMotorMaxPosition);

        boolean inRange = false;
        Range<Double> endRange = Range.create(-0.05, 0.05);

        while (inRange) {
            armMotorPower = armMotorPID.getOutput(armMotor.getCurrentPosition(), targetPosition);

            if (endRange.contains(armMotorPower)) {
                inRange = true;
            }

            armMotor.setPower(armMotorPower);
        }

        armMotor.setPower(0.0);
    }

    private enum Position {
        UP,
        HALFWAY,
        DOWN
    }
}
