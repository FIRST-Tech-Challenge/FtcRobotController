package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motors {


    public enum Type {
        LeftBack (0),
        LeftFront (1),
        RightFront (2),
        RightBack (3),
        Arm (4), // the arm that swings back and forth
        UpArm(5); // the arm that goes up and down

        private final int value;

        Type(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    private final DcMotor[] motors;

    private final int restingArmPosition;
    private final int reachingArmPosition;
    private final int restingUpArmPosition;
    private final int reachingUpArmPosition;


    public Motors(HardwareMap hardwareMap, boolean isAutonomous) {

        motors = new DcMotor[Type.values().length];

        motors[Type.LeftBack.getValue()] = hardwareMap.get(DcMotor.class, "0");
        motors[Type.LeftFront.getValue()] = hardwareMap.get(DcMotor.class, "1");
        motors[Type.RightFront.getValue()] = hardwareMap.get(DcMotor.class, "2");
        motors[Type.RightBack.getValue()] = hardwareMap.get(DcMotor.class, "3");
        motors[Type.Arm.getValue()] = hardwareMap.get(DcMotor.class, "4");
        motors[Type.UpArm.getValue()] = hardwareMap.get(DcMotor.class, "5");


        motors[Type.LeftBack.getValue()].setDirection(DcMotor.Direction.REVERSE);
        motors[Type.LeftFront.getValue()].setDirection(DcMotor.Direction.REVERSE);

        motors[Type.RightFront.getValue()].setDirection(DcMotor.Direction.FORWARD);
        motors[Type.RightBack.getValue()].setDirection(DcMotor.Direction.FORWARD);

        motors[Type.Arm.getValue()].setDirection(DcMotor.Direction.REVERSE);
        motors[Type.UpArm.getValue()].setDirection(DcMotor.Direction.FORWARD);

        if(isAutonomous) {

           motors[Type.LeftBack.getValue()].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           motors[Type.LeftFront.getValue()].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

           motors[Type.RightFront.getValue()].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           motors[Type.RightBack.getValue()].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




            motors[Type.LeftBack.getValue()].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[Type.LeftFront.getValue()].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motors[Type.RightFront.getValue()].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[Type.RightBack.getValue()].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



            motors[Type.RightFront.getValue()].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[Type.LeftBack.getValue()].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[Type.LeftFront.getValue()].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[Type.RightBack.getValue()].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motors[Type.Arm.getValue()].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[Type.UpArm.getValue()].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors[Type.Arm.getValue()].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[Type.UpArm.getValue()].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors[Type.Arm.getValue()].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // doesn't actually stop the motor from moving, just slows it down so it doesn't slam into the ground
        setArmPos(0);
        

        restingArmPosition = getArmPosition();
        reachingArmPosition = getArmPosition() + Constants.ARM_MAX_POSITION_OFFSET;

        restingUpArmPosition = getUpArmPosition();
        reachingUpArmPosition = getUpArmPosition() + Constants.UP_ARM_MAX_POSITION_OFFSET;
    }

    public void MoveMotor(Type motorNumber, double power) { //choose motor to move with type and move with power is 0-100

        double actualPower = power / 100;

        motors[motorNumber.getValue()].setPower(actualPower);
    }
    public int getArmPosition() {
        return motors[Type.Arm.getValue()].getCurrentPosition();
    }

    public int getRestingUpArmPosition(){return restingUpArmPosition;}

    public int getReachingUpArmPosition(){return reachingUpArmPosition;}

    public int getArmRestingPosition() {
        return restingArmPosition;
    }

    public int getArmReachingPosition() {
        return reachingArmPosition;
    }
    public int getUpArmPosition() {
        return motors[Type.UpArm.getValue()].getCurrentPosition();
    }

    public int getLeftFrontPosition() {
        BotTelemetry.addData("leftFrontposition", motors[Type.RightFront.getValue()].getCurrentPosition());
        return motors[Type.LeftFront.getValue()].getCurrentPosition();
    }

    public int getLeftBackPosition() {
        BotTelemetry.addData("leftBackPosition", motors[Type.LeftBack.getValue()].getCurrentPosition());
        return motors[Type.LeftBack.getValue()].getCurrentPosition();
    }
    public int getRightFrontPosition() {
        BotTelemetry.addData("rightFrontPosition", motors[Type.RightFront.getValue()].getCurrentPosition());
        return motors[Type.RightFront.getValue()].getCurrentPosition();
    }
    public int getRightBackPosition() {
        BotTelemetry.addData("rightBackPosition", motors[Type.RightBack.getValue()].getCurrentPosition());
        return motors[Type.RightBack.getValue()].getCurrentPosition();
    }

    public void resetDistance() {
        motors[Type.LeftFront.getValue()].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[Type.RightFront.getValue()].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors[Type.LeftFront.getValue()].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[Type.RightFront.getValue()].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setArmPos(int position){
        motors[Type.Arm.getValue()].setTargetPosition(position);
    }
    public void goTargetPosition() {
        motors[Type.Arm.getValue()].setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
