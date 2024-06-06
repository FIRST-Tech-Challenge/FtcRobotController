package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    DcMotor intake = null;
    Servo intakeHeight = null;
    Servo transferServo;
    DigitalChannel frontBreakBeam, backBreakBeam;
    Telemetry telemetry;
    CSCons.IntakeDirection intakeDirection = CSCons.IntakeDirection.OFF;

    int stackPosition = 0;
    PixelStatus pixelStatus;
    TransferServoStatus transferServoStatus;

    public enum PixelStatus{
        WAITING_FOR_PIXELS,
        BACK_PIXEL,
        HAS_PIXELS
    }

    public enum TransferServoStatus{
        UP,
        DOWN
    }

    public Intake(HardwareMap hardwareMap, Telemetry telemetry){

        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeHeight = hardwareMap.servo.get("intakeServo");
        transferServo = hardwareMap.servo.get("transfer");

        frontBreakBeam = hardwareMap.digitalChannel.get("breakBeam2");
        frontBreakBeam.setMode(DigitalChannel.Mode.INPUT);
        backBreakBeam = hardwareMap.digitalChannel.get("breakBeam1");
        backBreakBeam.setMode(DigitalChannel.Mode.INPUT);

        this.telemetry = telemetry;

    }

    public void init(){
        intakeHeight.setPosition(CSCons.intakeInit);
        transferServo.setPosition(CSCons.transferUp);
        transferServoStatus= TransferServoStatus.UP;
        updatePixelStatus();
    }

    public void update(){
        updatePixelStatus();
        switch (stackPosition) {
            case 0:
                intakeHeight.setPosition(CSCons.intakeBottom);
                break;
            case 1:
                intakeHeight.setPosition(CSCons.intake2);
                break;
            case 2:
                intakeHeight.setPosition(CSCons.intake3);
                break;
            case 3:
                intakeHeight.setPosition(CSCons.intake4);
                break;
            case 4:
                intakeHeight.setPosition(CSCons.intake5);
                break;
            case 5:
                intakeHeight.setPosition(CSCons.intakeAboveTop);
                break;
        }

    }

    public void raiseIntake(){
        if (stackPosition<5){
            stackPosition++;
        }
    }

    public void lowerIntake(){
        if (stackPosition>0){
            stackPosition--;
        }
    }

    protected void updatePixelStatus(){
        if (!frontBreakBeam.getState() && !backBreakBeam.getState()){
            pixelStatus = PixelStatus.HAS_PIXELS;
        } else if (!backBreakBeam.getState()){
            pixelStatus = PixelStatus.BACK_PIXEL;
        } else {
            pixelStatus = PixelStatus.WAITING_FOR_PIXELS;
        }
    }

    public void run(){
        intakeDirection = CSCons.IntakeDirection.ON;
        intake.setPower(CSCons.speed);
    }

    public void stop(){
        intakeDirection = CSCons.IntakeDirection.OFF;
        intake.setPower(0);
    }

    public void reverse(){
        intakeDirection = CSCons.IntakeDirection.BACKWARD;
        intake.setPower(-CSCons.speed);
    }


}
