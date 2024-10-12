package org.firstinspires.ftc.masters.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.components.Component;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class Intake implements Component {
    private HardwareMap hardwareMap = null;
    CSCons.DriveMode driveMode= CSCons.DriveMode.NORMAL;

    ElapsedTime elapsedTime = new ElapsedTime();

    DcMotor intake = null;
    Servo intakeHeight = null;
    Servo transferServo;

    CSCons.IntakeDirection intakeDirection;
    int stackPosition;
    boolean stackButtonPushed= false;
    CSCons.TransferStatus currentTransferStatus;

    Transfer transfer = null;
    Telemetry telemetry;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.transfer = Transfer.getInstance(hardwareMap, telemetry);
        this.telemetry=telemetry;
        initializeHardware();
    }
    public void initializeHardware(){

        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeHeight = hardwareMap.servo.get("intakeServo");
        transferServo = hardwareMap.servo.get("transfer");

        intakeHeight.setPosition(CSCons.intakeInit);
        transferServo.setPosition(CSCons.transferUp);
        intakeDirection= CSCons.IntakeDirection.OFF;
        stackPosition=0;
    }

    public void update(Gamepad gamepad1, CSCons.DriveMode driveMode, CSCons.OuttakeState outtakeState){

        if (driveMode== CSCons.DriveMode.HANG ){
            //setup hang
            initIntake();
            intakeControl(CSCons.IntakeDirection.OFF);

            this.driveMode= driveMode;
        } else {
            if (transfer.getCurrentTransferStatus()== CSCons.TransferStatus.CLOSE_FINGERS){
                stackPosition = 5;
            }
            if (transfer.getCurrentTransferStatus()== CSCons.TransferStatus.DONE && currentTransferStatus!= transfer.getCurrentTransferStatus()){
                intakeControl(CSCons.IntakeDirection.BACKWARD);
                stackPosition=3;
                elapsedTime = new ElapsedTime();
            }

            if (currentTransferStatus == CSCons.TransferStatus.DONE && elapsedTime!=null && elapsedTime.milliseconds()>3000){
                intakeControl(CSCons.IntakeDirection.OFF);
                elapsedTime =null;
            }


            if (outtakeState!= CSCons.OuttakeState.ReadyToDrop) {
                intakeControl(gamepad1);
                stackControl(gamepad1);
            }

            setIntakeHeight(stackPosition);
            currentTransferStatus = transfer.getCurrentTransferStatus();
        }

    }


    protected void intakeControl(Gamepad gamepad){
        //right stick up intake on
        if (gamepad.right_stick_y<-0.5 && Math.abs(gamepad.right_stick_x)<0.5){
            intakeDirection= CSCons.IntakeDirection.ON;
        }
        //right stick down intake backward
        if (gamepad.right_stick_y>0.5 && Math.abs(gamepad.right_stick_x)<0.5){
            stackPosition=3;
            intakeDirection= CSCons.IntakeDirection.BACKWARD;
        }
        //right stick right intake off
        if (gamepad.right_stick_x>0.5 && Math.abs(gamepad.right_stick_y)<0.5){
            intakeDirection= CSCons.IntakeDirection.OFF;
        }
        intakeControl(intakeDirection);
    }

    protected  void intakeControl(CSCons.IntakeDirection intakeDirection){
        this.intakeDirection= intakeDirection;
        if (intakeDirection== CSCons.IntakeDirection.ON){
            intake.setPower(CSCons.speed);

        }
        if (intakeDirection== CSCons.IntakeDirection.BACKWARD){
            intake.setPower(-CSCons.speed);
        }
        if (intakeDirection== CSCons.IntakeDirection.OFF){
            intake.setPower(0);
        }
    }

    protected void stackControl(Gamepad gamepad){
        //intake top
        if (gamepad.dpad_up){
            if (!stackButtonPushed){
                stackPosition=5;
            }
            stackButtonPushed = true;
        //intake bottom
        } else if (gamepad.dpad_down){
            if (!stackButtonPushed){
                stackPosition=0;
            }
            stackButtonPushed= true;

            //intake down by 1
        } else if (gamepad.dpad_left){
            if (!stackButtonPushed && stackPosition>0){
                stackPosition--;
            }
            stackButtonPushed = true;

            //intake up by 1
        } else if (gamepad.dpad_right){
            if (!stackButtonPushed && stackPosition<5){
                stackPosition++;
            }
            stackButtonPushed = true;
        } else {
            stackButtonPushed= false;
        }
    }


    public void initIntake(){
        intakeHeight.setPosition(CSCons.intakeInit);
    }

    public void setIntakeHeight(int stackPosition){

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



}
