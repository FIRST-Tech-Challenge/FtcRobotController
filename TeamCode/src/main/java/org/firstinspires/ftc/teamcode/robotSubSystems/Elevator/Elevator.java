package org.firstinspires.ftc.teamcode.robotSubSystems.Elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    public static DcMotor rightMotor;
    public static DcMotor leftMotor;

    public static double currentElevPosL;
    public static double currentElevPosR;
    public static ElevatorState lastElevState;

    public static int encoderResetValL;
    public static int encoderResetValR;
    public static void init(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotor.class, "1");
        leftMotor = hardwareMap.get(DcMotor.class, "0");
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static void operatee(ElevatorState state, double gamepadVal) {
        switch (state) {
            case INTAKE:
                leftMotor.setTargetPosition(ElevatorConstants.IntakePos);
                rightMotor.setTargetPosition(0);
                rightMotor.setPower(0.5);
                leftMotor.setPower(0.5);
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentElevPosL = getElevatorPosL();
                currentElevPosR = getElevatorPosR();
                break;


            case SPECIMEN:
                leftMotor.setTargetPosition(ElevatorConstants.SpecimenPos);
                rightMotor.setTargetPosition(ElevatorConstants.SpecimenPos);
                //power is elevator speed
                rightMotor.setPower(0.9);
                leftMotor.setPower(0.9);
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentElevPosL = getElevatorPosL();
                currentElevPosR = getElevatorPosR();
                break;
            case MANUAL:
                //the number is the elevator speed (PID already in the settargetposition func)
                leftMotor.setTargetPosition((int) (currentElevPosL + gamepadVal * 100));
                rightMotor.setTargetPosition((int) (currentElevPosR + gamepadVal * 100));
                currentElevPosL = getElevatorPosL();
                currentElevPosR = getElevatorPosR();
                break;
        }
    }

    public static void operate(double val) {
        rightMotor.setPower(val);
        leftMotor.setPower(val);
    }


    public static void operateTest(int Val) {
        leftMotor.setTargetPosition(Val);

    }

    public static double getElevatorPosL() {
        return leftMotor.getCurrentPosition() - encoderResetValL;
    }
    public static double getElevatorPosR() {
        return rightMotor.getCurrentPosition() - encoderResetValL;
    }

    public static void resetEncoder(){
        encoderResetValR = leftMotor.getCurrentPosition();
        encoderResetValL = leftMotor.getCurrentPosition();
    }
}
