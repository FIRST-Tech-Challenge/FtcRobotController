package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// a utils class to load all drive motors and group motor functions
public interface MotorUtils {
    Motor frontRightDrive = new Motor("frontright", true, true);
    Motor backRightDrive = new Motor("backright", false, true);
    Motor frontLeftDrive = new Motor("frontleft", true, true);
    Motor backLeftDrive = new Motor("backleft", false, true);

    enum REVERSED{
        RIGHT, LEFT
    }

    default void reverseMotorSide(REVERSED rev){
        if (rev==REVERSED.RIGHT){
            frontRightDrive.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
            backRightDrive.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (rev==REVERSED.LEFT) {
            frontLeftDrive.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftDrive.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    //utils for using encoder
    default void stopMotors() {
        frontRightDrive.stopMotor();
        backRightDrive.stopMotor();
        frontLeftDrive.stopMotor();
        backLeftDrive.stopMotor();
    }

    default void updateMotors(DcMotorEx motorEx, int CUR){
        frontRightDrive.setPower(UpdatePowerTypes.startEndUpdatePower(motorEx, CUR));
        backRightDrive.setPower(UpdatePowerTypes.startEndUpdatePower(motorEx, CUR));
        frontLeftDrive.setPower(UpdatePowerTypes.startEndUpdatePower(motorEx, CUR));
        backLeftDrive.setPower(UpdatePowerTypes.startEndUpdatePower(motorEx, CUR));
    }
    default void updateMotorsSideways(DcMotorEx motorEx, int CUR){
        frontRightDrive.setPower(-UpdatePowerTypes.startEndUpdatePower(motorEx, CUR));
        backRightDrive.setPower(UpdatePowerTypes.startEndUpdatePower(motorEx, CUR));
        frontLeftDrive.setPower(UpdatePowerTypes.startEndUpdatePower(motorEx, CUR));
        backLeftDrive.setPower(-UpdatePowerTypes.startEndUpdatePower(motorEx, CUR));
    }

    default void cleanupMotors() {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    default void setModeAllDrive(DcMotor.RunMode mode) {
        frontRightDrive.setMode(mode);
        frontLeftDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }
}
