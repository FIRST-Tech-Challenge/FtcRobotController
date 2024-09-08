package org.firstinspires.ftc.masters.old;

import static org.firstinspires.ftc.masters.old.CSCons.transferPush;
import static org.firstinspires.ftc.masters.old.CSCons.transferUp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class Transfer {

    private CSCons.TransferStatus currentTransferStatus;
    private static Transfer singleInstance= null;

    private final DigitalChannel frontBreakBeam;
    private final DigitalChannel backBreakBeam;

    private ElapsedTime elapsedTime = null;

    private final Servo transferServo;
    boolean pickupOverride = false;
    Telemetry telemetry;

    private Transfer(HardwareMap hardwareMap, Telemetry telemetry){
        currentTransferStatus = CSCons.TransferStatus.WAITING_FOR_PIXELS;
        transferServo = hardwareMap.servo.get("transfer");
        frontBreakBeam = hardwareMap.digitalChannel.get("breakBeam2");
        frontBreakBeam.setMode(DigitalChannel.Mode.INPUT);
        backBreakBeam = hardwareMap.digitalChannel.get("breakBeam1");
        backBreakBeam.setMode(DigitalChannel.Mode.INPUT);
        this.telemetry = telemetry;

    }

    public static Transfer getInstance(HardwareMap hardwareMap, Telemetry telemetry){
        if (singleInstance==null){
            singleInstance = new Transfer(hardwareMap, telemetry);
        }
        return singleInstance;
    }

    public void update(Gamepad gamepad, CSCons.OuttakeState outtakeState){

        if ((has2Pixels() || pickupOverride || currentTransferStatus== CSCons.TransferStatus.DONE) &&  outtakeState== CSCons.OuttakeState.ReadyToTransfer) {

            telemetry.addData("override", pickupOverride);
            if (currentTransferStatus == CSCons.TransferStatus.WAITING_FOR_PIXELS){
                transferServo.setPosition(transferPush);
                elapsedTime = new ElapsedTime();
                currentTransferStatus = CSCons.TransferStatus.MOVE_ARM;
            }
            if (currentTransferStatus == CSCons.TransferStatus.MOVE_ARM && elapsedTime != null && elapsedTime.milliseconds() > CSCons.TransferStatus.MOVE_ARM.getWaitTime()) {
                currentTransferStatus = CSCons.TransferStatus.MOVE_OUTTAKE;
                elapsedTime = new ElapsedTime();
            }
            if (currentTransferStatus == CSCons.TransferStatus.MOVE_OUTTAKE && elapsedTime != null && elapsedTime.milliseconds() > CSCons.TransferStatus.MOVE_OUTTAKE.getWaitTime()) {
                currentTransferStatus = CSCons.TransferStatus.CLOSE_FINGERS;
                elapsedTime = new ElapsedTime();
            }
            if (currentTransferStatus == CSCons.TransferStatus.CLOSE_FINGERS && elapsedTime.milliseconds() > CSCons.TransferStatus.CLOSE_FINGERS.getWaitTime()) {
                transferServo.setPosition(transferUp);
                elapsedTime = null;
                if (gamepad!=null) {
                    gamepad.rumble(1500);
                }
                currentTransferStatus= CSCons.TransferStatus.DONE;
                pickupOverride = false;
            }

        } else{
            currentTransferStatus= CSCons.TransferStatus.WAITING_FOR_PIXELS;
            transferServo.setPosition(transferUp);
            pickupOverride= false;
        }

        telemetry.addData("transfer state", currentTransferStatus);

    }

    public CSCons.TransferStatus getCurrentTransferStatus(){
        return currentTransferStatus;
    }

    public void setCurrentTransferStatus(CSCons.TransferStatus transferStatus) {
        if (this.currentTransferStatus == CSCons.TransferStatus.WAITING_FOR_PIXELS && transferStatus == CSCons.TransferStatus.MOVE_ARM){
            if (elapsedTime ==null){
                elapsedTime = new ElapsedTime();
            }
            transferServo.setPosition(transferUp);
        }

        this.currentTransferStatus = transferStatus;
    }

    public boolean has2Pixels(){
        return !frontBreakBeam.getState() && !backBreakBeam.getState();
    }

    public void setPickupOverride(boolean transferOverride) {
        this.pickupOverride = transferOverride;
    }

    public void setTransferServoUp(){
        this.transferServo.setPosition(transferUp);
    }
}
