package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Range;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.MiniPID;

public class ArmSubsystem {
    //Variable Declarations
    private DcMotor arm;
    private double armMotorPower;
    private int armZeroPosition;

    private TouchSensor limitSwitch;

    MiniPID armMotorPID;

    ElapsedTime runtime;
    Telemetry telemetry;

    public enum Position {
        UP,
        HALFWAY,
        DOWN
    }

    //Constructor Class
    public ArmSubsystem(DcMotor arm, TouchSensor limitSwitch, ElapsedTime runtime, Telemetry telemetry) {
        this.arm = arm;
        this.limitSwitch = limitSwitch;
        this.runtime = runtime;
        this.telemetry = telemetry;
        initialize();
    }

    //Initializes arm motor and PID controller. Finds starting position.
    public void initialize() {
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotorPID = new MiniPID(Constants.armP, Constants.armI, Constants.armD);
        armMotorPID.setOutputLimits(-Constants.teleOPArmPower, Constants.teleOPArmPower);
        zeroPosition();
    }

    public void autoPositionArm(Position position) {
        armMotorPID.reset();
        int targetPosition;
        switch (position) {
            case UP:
                targetPosition = armZeroPosition + Constants.armMotor120Position;
                break;

            case HALFWAY:
                targetPosition = armZeroPosition + Constants.armMotor90Position;
                break;

            case DOWN:
                targetPosition = armZeroPosition;
                break;

            default:
                return;
        }

        boolean motorActive = true;
        double startTime = runtime.seconds();
        while (motorActive) {
            armMotorPower = armMotorPID.getOutput(arm.getCurrentPosition(), targetPosition);
            if (-0.05 < armMotorPower && armMotorPower < 0.05) {
                motorActive = false;
            }
            if (runtime.seconds() - startTime > 8) {
                motorActive = false;
            }
            arm.setPower(armMotorPower);
        }
        arm.setPower(0.0);
    }

    private void zeroPosition() {
        while (!limitSwitch.isPressed()) {
            arm.setPower(Constants.armZeroingPower);
        }
        arm.setPower(0.0);
        armZeroPosition = (int) (arm.getCurrentPosition() + 56);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(armZeroPosition);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Rotates the arm based off DPad input.
    public void ManualPositionArm(boolean gamepad2_DPadLeft, boolean gamepad2_DPadRight) {
        Range<Integer> rangeOfMotion = Range.create(armZeroPosition, armZeroPosition + Constants.armMotor120Position);
        if (rangeOfMotion.contains(arm.getCurrentPosition())) {
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
        else if (arm.getCurrentPosition() < armZeroPosition) {
            armMotorPower = Constants.teleOPArmPower;
        }

        else {
            armMotorPower = -Constants.teleOPArmPower;
        }
    }

    //Moves the arm to the desired angle utilizing the arm PID.
    public void autoCustomPositionArm (double desiredAngle) {
        armMotorPID.reset();
        double targetPosition = (desiredAngle / 360) * Constants.armMotorCPR * Constants.armGearRatio;
        com.qualcomm.robotcore.util.Range.clip(targetPosition, armZeroPosition, armZeroPosition + Constants.armMotor120Position);

        boolean motorActive = true;
        Range<Double> endRange = Range.create(-0.05, 0.05);

        double startTime = runtime.seconds();
        while (motorActive) {
            armMotorPower = armMotorPID.getOutput(arm.getCurrentPosition(), targetPosition);

            if (endRange.contains(armMotorPower)) {
                motorActive = false;
            }

            if (runtime.seconds() - startTime > 8) {
                motorActive = false;
            }

            arm.setPower(armMotorPower);
        }

        arm.setPower(0.0);
    }

    public int getArmPosition() {
        return arm.getCurrentPosition();
    }
}
