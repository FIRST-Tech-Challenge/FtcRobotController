package com.kalipsorobotics.actions.drivetrain;

import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveAction {
    OpModeUtilities opModeUtilities;

    DriveTrain driveTrain;
    private final DcMotor fLeft, fRight, bLeft, bRight;
    private final DcMotor backEncoder;
    private final DcMotor rightEncoder;
    private final DcMotor leftEncoder;

    private final double[] driveTrainPower = new double[4];

    public DriveAction(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.fLeft = driveTrain.getfLeft();
        this.fRight = driveTrain.getfRight();
        this.bLeft = driveTrain.getbLeft();
        this.bRight = driveTrain.getbRight();
        this.rightEncoder = driveTrain.getRightEncoder();
        this.leftEncoder = driveTrain.getLeftEncoder();
        this.backEncoder = driveTrain.getBackEncoder();
    }

    public double[] calculatePower(Gamepad gamepad) {
        //negative because gamepad y is flip
        double forward = -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;
        double strafe = gamepad.left_stick_x;
        double fLeftPower = (forward + strafe + turn);
        double fRightPower = (forward - strafe - turn);
        double bLeftPower = (forward - strafe + turn);
        double bRightPower = (forward + strafe - turn);

//        double fLeftPower = powerX + powerY + powerAngle;
//        double fRightPower = powerX - powerY - powerAngle;
//
//        double bLeftPower = powerX - powerY + powerAngle;
//        double bRightPower = powerX + powerY - powerAngle;


        double absMaxPower = MathFunctions.maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
        if (absMaxPower > 1) {
            fLeftPower = fLeftPower / absMaxPower;
            fRightPower = fRightPower / absMaxPower;
            bLeftPower = bLeftPower / absMaxPower;
            bRightPower = bRightPower / absMaxPower;
        }
        double[] driveTrainPowers = {fLeftPower, fRightPower, bLeftPower, bRightPower};
        return driveTrainPowers;
    }

    public void move(Gamepad gamepad) {
        fLeft.setPower(calculatePower(gamepad)[0]);
        fRight.setPower(calculatePower(gamepad)[1]);
        bLeft.setPower(calculatePower(gamepad)[2]);
        bRight.setPower(calculatePower(gamepad)[3]);
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }
}
