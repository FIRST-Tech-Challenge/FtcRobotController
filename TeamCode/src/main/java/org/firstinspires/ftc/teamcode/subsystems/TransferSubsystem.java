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

    public int TRANSFER_TARGET = 760;
    public int transferOffset = 0;

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
        while(transferTouchBottom.getState() && !robot.operatorCancel) {
            dropTransfer();
        }
    }

    public void raiseTransfer() {
        while(transferTouchTop.getState() && !robot.operatorCancel) {
            transfer.setPower(0.45);
        }
        transfer.setPower(0);
    }

    public void dropTransfer() {
        double initialTime = System.currentTimeMillis();
        while(transferTouchBottom.getState() && !robot.operatorCancel && (System.currentTimeMillis() - initialTime) < 3000) {
            transfer.setPower(-0.25);
        }
        transfer.setPower(0);
    }

}

