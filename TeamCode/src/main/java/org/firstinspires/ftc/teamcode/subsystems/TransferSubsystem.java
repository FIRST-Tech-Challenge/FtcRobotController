package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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

    public double getTransferCurrent(){return robot.transfer.getCurrent(CurrentUnit.MILLIAMPS);}

    public void initTransferPos() {
        teleDropTransfer();
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void autoRaiseTransfer() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                transfer.setTargetPosition(TRANSFER_TARGET);
                transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                transfer.setPower(0.65);
                robot.setTransferState(UpliftRobot.TransferState.MOVING);
                double initialTime = System.currentTimeMillis();
                while(transfer.isBusy() && !robot.operatorCancel && getTransferCurrent() < 1700 && robot.opMode.opModeIsActive()) {
                    robot.safeSleep(5);
                }
                transfer.setPower(0);
                transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.setTransferState(UpliftRobot.TransferState.UP);
            }
        }).start();
    }

    public void teleRaiseTransfer() {
        transfer.setTargetPosition(TRANSFER_TARGET);
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setPower(0.65);
        robot.setTransferState(UpliftRobot.TransferState.MOVING);
        double initialTime = System.currentTimeMillis();
        while(transfer.isBusy() && !robot.operatorCancel && getTransferCurrent() < 1700 && robot.opMode.opModeIsActive()) {
            robot.safeSleep(5);
        }
        transfer.setPower(0);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setTransferState(UpliftRobot.TransferState.UP);
    }

    public void autoDropTransfer() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                robot.setTransferState(UpliftRobot.TransferState.MOVING);
                transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double initialTime = System.currentTimeMillis();
                while(transferTouchBottom.getState() && !robot.operatorCancel && (System.currentTimeMillis() - initialTime) < 3000) {
                    transfer.setPower(0.65);
                }
                transfer.setPower(0);
                robot.setTransferState(UpliftRobot.TransferState.DOWN);
            }
        }).start();
    }

    public void teleDropTransfer() {
        robot.setTransferState(UpliftRobot.TransferState.MOVING);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double initialTime = System.currentTimeMillis();
        while(transferTouchBottom.getState() && !robot.operatorCancel) {
            transfer.setPower(0.65);
        }
        transfer.setPower(0);
        robot.setTransferState(UpliftRobot.TransferState.DOWN);
    }

}

