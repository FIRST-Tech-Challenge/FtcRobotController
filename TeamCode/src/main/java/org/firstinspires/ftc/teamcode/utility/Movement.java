package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Movement {

    public int motor_ticks;
    public static enum Direction {
        FRONT,
        BACK,
        LEFT,
        RIGHT
    }
    private DcMotor lfDrive;
    private DcMotor rfDrive;
    private DcMotor lbDrive;
    private DcMotor rbDrive;
    public Movement(DcMotor leftFrontDrive,DcMotor rightFrontDrive,
                    DcMotor leftBackDrive, DcMotor rightBackDrive){
        lfDrive = leftFrontDrive;
        rfDrive = rightFrontDrive;
        lbDrive = leftBackDrive;
        rbDrive = rightBackDrive;
    }

    private void initMovement(){
        motor_ticks = 0;
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        //lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        //rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        //lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        //rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
    }

    public void Forward(int ticks){
        initMovement();
        lfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(0.5);
        rfDrive.setPower(0.5);
        lbDrive.setPower(0.5);
        rbDrive.setPower(0.5);
        while(lfDrive.getCurrentPosition()<= (ticks-2)){

        }
    }
    public void Backwards(int ticks){
        initMovement();
        lfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(0.5);
        rfDrive.setPower(0.5);
        lbDrive.setPower(0.5);
        rbDrive.setPower(0.5);
        while((lfDrive.getCurrentPosition()*-1)<= (ticks-2)){

        }
    }


    public void Left(int ticks){
        initMovement();
        lfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(0.5);
        rfDrive.setPower(0.5);
        lbDrive.setPower(0.5);
        rbDrive.setPower(0.5);
        while((lfDrive.getCurrentPosition()*-1)<= (ticks-2)){

        }
    }

    public void Right(int ticks){
        initMovement();
        lfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(0.5);
        rfDrive.setPower(0.5);
        lbDrive.setPower(0.5);
        rbDrive.setPower(0.5);
        while(lfDrive.getCurrentPosition()<= (ticks-2)) {

        }
    }

    public void Rotate(int degrees){

    }
}
