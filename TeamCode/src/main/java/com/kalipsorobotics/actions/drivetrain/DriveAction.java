package com.kalipsorobotics.actions.drivetrain;

import android.util.Log;

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

    public double[] calculatePower(double gamepadx, double gamepady, Gamepad gamepad) {
        //negative because gamepad y is flip
        double forward = -gamepady * -gamepady * -gamepady; //cube so slower is slower and faster is faster
        double turn = gamepad.right_stick_x * gamepad.right_stick_x * gamepad.right_stick_x;
        double strafe = gamepadx * gamepadx * gamepadx;

        Log.d("drive", "forward " + forward);
        Log.d("drive", "turn " + turn);
        Log.d("drive", "strafe " + strafe);

        double fLeftPower = (forward + strafe + turn);
        double fRightPower = (forward - strafe - turn);
        double bLeftPower = (forward - strafe + turn);
        double bRightPower = (forward + strafe - turn);

//        double fLeftPower = powerX + powerY + powerAngle;
//        double fRightPower = powerX - powerY - powerAngle;
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
    public double[] calculatePower(Gamepad gamepad) {
        //negative because gamepad y is flip
        double forward = -gamepad.left_stick_y * -gamepad.left_stick_y * -gamepad.left_stick_y; //cube so slower is slower and faster is faster
        double turn = gamepad.right_stick_x * gamepad.right_stick_x * gamepad.right_stick_x;
        double strafe = gamepad.left_stick_x * gamepad.left_stick_x * gamepad.left_stick_x;

        Log.d("drive", "forward " + forward);
        Log.d("drive", "turn " + turn);
        Log.d("drive", "strafe " + strafe);

        double fLeftPower = (forward + strafe + turn);
        double fRightPower = (forward - strafe - turn);
        double bLeftPower = (forward - strafe + turn);
        double bRightPower = (forward + strafe - turn);

//        double fLeftPower = powerX + powerY + powerAngle;
//        double fRightPower = powerX - powerY - powerAngle;
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

        double[] driveTrainPower = calculatePower(gamepad);

        fLeft.setPower(driveTrainPower[0]);
        fRight.setPower(driveTrainPower[1]);
        bLeft.setPower(driveTrainPower[2]);
        bRight.setPower(driveTrainPower[3]);

        Log.d("drive", "fLeft power " + driveTrainPower[0]);
        Log.d("drive", "fRight power " + driveTrainPower[1]);
        Log.d("drive", "bLeft power " + driveTrainPower[2]);
        Log.d("drive", "bRight power " + driveTrainPower[3]);

    }
    public void moveWithXYValues(double x, double y) {
        double[] driveTrainPower = calculatePower(x, y);

        fLeft.setPower(driveTrainPower[0]);
        fRight.setPower(driveTrainPower[1]);
        bLeft.setPower(driveTrainPower[2]);
        bRight.setPower(driveTrainPower[3]);

        Log.d("drive", "fLeft power " + driveTrainPower[0]);
        Log.d("drive", "fRight power " + driveTrainPower[1]);
        Log.d("drive", "bLeft power " + driveTrainPower[2]);
        Log.d("drive", "bRight power " + driveTrainPower[3]);
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }
}
