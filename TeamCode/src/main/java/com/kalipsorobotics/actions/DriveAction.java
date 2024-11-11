package com.kalipsorobotics.actions;

import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveAction {
    OpModeUtilities opModeUtilities;

    DriveTrain driveTrain;
    private final DcMotor fLeft, fRight, bLeft, bRight;
    private final DcMotor backEncoder;
    private final DcMotor rightEncoder;
    private final DcMotor leftEncoder;

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

    public void move(Gamepad gamepad) {
        double forward = -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;
        double strafe = gamepad.left_stick_x;
        fLeft.setPower(forward + turn + strafe);
        fRight.setPower(forward - turn - strafe);
        bLeft.setPower(forward + turn - strafe);
        bRight.setPower(forward - turn + strafe);
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }
}
