package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class Drivetrain {

    DcMotor fLeft, fRight, bLeft, bRight;

    LinearOpMode opMode;

    public PIDController straightController = new PIDController("straight", 0.005, 0.0000015, 0.8, false);
    public PIDController fLeftMecanumController = new PIDController("fl mecanum", 0.005, 0.0000005, 0.4, true);

    public Drivetrain(Robot robot) {
        this.fLeft = robot.fLeft;
        this.fRight = robot.fRight;
        this.bLeft = robot.bLeft;
        this.bRight = robot.bRight;

        this.opMode = robot.opMode;
    }

    public void setMotorPower(final DrivetrainPowers drivetrainPowers) {
        setMotorPower(drivetrainPowers.fLeftPower, drivetrainPowers.fRightPower, drivetrainPowers.bLeftPower, drivetrainPowers.bRightPower);
    }

    public void setMotorPower(double fLeft, double fRight, double bLeft, double bRight) {
        this.fLeft.setPower(fLeft);
        this.bRight.setPower(bRight);
        this.bLeft.setPower(bLeft);
        this.fRight.setPower(fRight);
    }

    public void setMotorPower(DcMotor dcMotor, double power) {
        dcMotor.setPower(power);
    }

    public DrivetrainPowers straightParallelPowerPISequence(GenericState conditionState, DrivetrainState state, double maxPower) {

        double power;

        if (conditionState.isDone()) {
            if (!state.isDone() && state.getOpMode().opModeIsActive()) {

                power = straightController.calculatePID(state.getCurrentTargetTicks(0), state.getCurrentTargetTicks(0));

                //cap power
                power = Range.clip(power, -1 * maxPower, maxPower);

                return new DrivetrainPowers(power, power, power, power);

            }
        }

        return new DrivetrainPowers(0, 0, 0, 0);
    }

}