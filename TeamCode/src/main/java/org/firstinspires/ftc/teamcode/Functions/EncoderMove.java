package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;

public class EncoderMove {


    public DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    public Move move;


    public EncoderMove(DcMotor _FL, DcMotor _FR,DcMotor _BL, DcMotor _BR) {

        leftMotor = _FL;
        rightMotor = _FR;
        leftMotorBack = _BL;
        rightMotorBack = _BR;
    }

    /**
     * Initialize the variables that store the current position of the encoders.
     */
    public int leftPos;
    public int rightPos;
    public int leftBackPos;
    public int rightBackPos;
    public int equal;

    public int rightTarget1;

    public void reset(){
        leftPos = 50;
        rightPos = 50;
        leftBackPos = 50;
        rightBackPos = 50;
    }

    // private void drive(int leftTarget, int rightTarget, int leftBackTarget, int rightBackTarget, double speed)
    public boolean drive(double speed){

        rightTarget1=50;
//        equal = rightTarget*300;


        leftPos += 50;
        rightPos += 50;
        leftBackPos += 50;
        rightBackPos += 50;


        /**
         * Set the target position, and set the mode to run to position.
         * This will make encoders move until they get to the target position.
         */
        leftMotor.setTargetPosition(leftPos);
        rightMotor.setTargetPosition(rightPos);
        leftMotorBack.setTargetPosition(leftBackPos);
        rightMotorBack.setTargetPosition(rightBackPos);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
        leftMotorBack.setPower(speed);
        rightMotorBack.setPower(speed);

        return true;

    }
    public boolean driveBack(double speed){

        rightTarget1=50;
//        equal = rightTarget*300;


        leftPos += 50;
        rightPos += 50;
        leftBackPos += 50;
        rightBackPos += 50;


        /**
         * Set the target position, and set the mode to run to position.
         * This will make encoders move until they get to the target position.
         */
        leftMotor.setTargetPosition(-leftPos);
        rightMotor.setTargetPosition(-rightPos);
        leftMotorBack.setTargetPosition(-leftBackPos);
        rightMotorBack.setTargetPosition(-rightBackPos);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
        leftMotorBack.setPower(speed);
        rightMotorBack.setPower(speed);

        return true;

    }
    public boolean driveSlideRight(double speed){

        rightTarget1=50;
//        equal = rightTarget*300;


        leftPos += 50;
        rightPos += 50;
        leftBackPos += 50;
        rightBackPos += 50;



        /**
         * Set the target position, and set the mode to run to position.
         * This will make encoders move until they get to the target position.
         */
        leftMotor.setTargetPosition(leftPos);

        rightMotor.setTargetPosition(-rightPos);
        leftMotorBack.setTargetPosition(-leftBackPos);
        rightMotorBack.setTargetPosition(rightBackPos);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
        leftMotorBack.setPower(speed);
        rightMotorBack.setPower(speed);

        return true;

    }
    public boolean driveSlideLeft(double speed){

        rightTarget1=50;
//        equal = rightTarget*300;


        leftPos += 50;
        rightPos += 50;
        leftBackPos += 50;
        rightBackPos += 50;


        /**
         * Set the target position, and set the mode to run to position.
         * This will make encoders move until they get to the target position.
         */
        leftMotor.setTargetPosition(-leftPos);
        rightMotor.setTargetPosition(rightPos);
        leftMotorBack.setTargetPosition(leftBackPos);
        rightMotorBack.setTargetPosition(-rightBackPos);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
        leftMotorBack.setPower(speed);
        rightMotorBack.setPower(speed);

        return true;

    }


    public boolean driveRotateLeft(double speed){

        rightTarget1=50;
//        equal = rightTarget*300;


        leftPos += 50;
        rightPos += 50;
        leftBackPos += 50;
        rightBackPos += 50;


        /**
         * Set the target position, and set the mode to run to position.
         * This will make encoders move until they get to the target position.
         */
        leftMotor.setTargetPosition(-leftPos);
        rightMotor.setTargetPosition(rightPos);
        leftMotorBack.setTargetPosition(-leftBackPos);
        rightMotorBack.setTargetPosition(rightBackPos);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
        leftMotorBack.setPower(speed);
        rightMotorBack.setPower(speed);

        return true;

    }

    public boolean driveRotateRight(double speed){

        rightTarget1=50;
//        equal = rightTarget*300;


        leftPos += 50;
        rightPos += 50;
        leftBackPos += 50;
        rightBackPos += 50;


        /**
         * Set the target position, and set the mode to run to position.
         * This will make encoders move until they get to the target position.
         */
        leftMotor.setTargetPosition(leftPos);
        rightMotor.setTargetPosition(-rightPos);
        leftMotorBack.setTargetPosition(leftBackPos);
        rightMotorBack.setTargetPosition(-rightBackPos);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
        leftMotorBack.setPower(speed);
        rightMotorBack.setPower(speed);

        return true;

    }

}