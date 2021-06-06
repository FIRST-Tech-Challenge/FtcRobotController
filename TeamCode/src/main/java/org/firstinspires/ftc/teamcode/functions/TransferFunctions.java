package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.UpliftRobot;

public class TransferFunctions {

    UpliftRobot robot;
    DcMotor transfer;
    DigitalChannel transferTouchBottom;
    DigitalChannel transferTouchTop;

    public int TRANSFER_TARGET = -735;

    public TransferFunctions(UpliftRobot robot){
        this.robot = robot;
        this.transfer = robot.transfer;
        this.transferTouchBottom = robot.digitalTouchBottom;
        this.transferTouchTop = robot.digitalTouchTop;
    }

    public void setTransferPower(double power) {
        transfer.setPower(power);
    }

    public boolean getTransferTouchBottom() {
        return !transferTouchBottom.getState();
    }

    public double getTransferPower() {
        return transfer.getPower();
    }

    public double getTransferCurrent() {
        return robot.transfer.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public void initTransferPos() {
        dropTransfer();
    }

    public void raiseTransfer() {
        transfer.setTargetPosition(TRANSFER_TARGET);
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setPower(0.65);
        double initialTime = System.currentTimeMillis();
        while(transfer.isBusy() && !robot.operatorCancel && robot.opMode.opModeIsActive() && robot.opMode.gamepad2.right_stick_y < 0.25) {
            robot.safeSleep(5);
        }
        transfer.setPower(0);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void dropTransfer() {
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double initialTime = System.currentTimeMillis();
        while(transferTouchBottom.getState() && !robot.operatorCancel) {
            transfer.setPower(0.65);
        }
        transfer.setPower(0);
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
