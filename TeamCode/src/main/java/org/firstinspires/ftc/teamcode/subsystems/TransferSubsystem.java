package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Subsystem;

public class TransferSubsystem extends Subsystem {

    private UpliftRobot robot;
    public DcMotor transfer;
    public DigitalChannel transferTouchBottom;
    public DigitalChannel transferTouchTop;

    public int TRANSFER_TARGET = -735;
//    public int transferOffset = 0;

    public TransferSubsystem(UpliftRobot robot){
        super(robot);
        this.robot = robot;
        this.transfer = robot.transfer;
        this.transferTouchBottom = robot.digitalTouchBottom;
        this.transferTouchTop = robot.digitalTouchTop;
    }
    @Override
    public void enable() {

    }

    @Override
    public void disable() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void safeDisable() {
        transfer.setPower(0);
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

    public void initTransferPos() {
        dropTransfer();
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

//    public void resetTransfer() {
//        double initialTime = System.currentTimeMillis();
//        while(transferTouchBottom.getState()) {
//            transfer.setPower(-0.25);
//        }
//        transfer.setPower(0);
//        transferOffset = transfer.getCurrentPosition();
//    }

//    public void raiseTransfer() {
//        transfer.setTargetPosition(transferOffset + TRANSFER_TARGET);
//    }

    public void raiseTransfer() {
//        transfer.setTargetPosition(TRANSFER_TARGET + transferOffset);
//        transfer.setPower(0.25);
//        while(transfer.getCurrentPosition() > (TRANSFER_TARGET + transferOffset)) {
//            transfer.setPower(-0.1);//10,3,0,0
//        }
//        transfer.setPower(0);
        transfer.setTargetPosition(TRANSFER_TARGET);
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setPower(0.35);
        double initialTime = System.currentTimeMillis();
        while(transfer.isBusy() && System.currentTimeMillis() - initialTime < 3000) {
            robot.safeSleep(5);
        }
        transfer.setPower(0);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void dropTransfer() {
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double initialTime = System.currentTimeMillis();
        while(transferTouchBottom.getState() && !robot.operatorCancel && (System.currentTimeMillis() - initialTime) < 3000) {
            transfer.setPower(0.35);
        }
        transfer.setPower(0);
//        transferOffset = transfer.getCurrentPosition();
    }

}

