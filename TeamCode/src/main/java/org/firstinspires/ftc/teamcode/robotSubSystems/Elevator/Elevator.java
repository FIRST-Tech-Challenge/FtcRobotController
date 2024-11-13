package org.firstinspires.ftc.teamcode.robotSubSystems.Elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    private static DcMotor rightMotor;
    private static DcMotor leftMotor;

    public static double currentElevPos;
    public static ElevatorState lastElevState;

    public static int encoderResetValL;
    public static int encoderResetValR;
    public static void init(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotor.class, "1");
        leftMotor = hardwareMap.get(DcMotor.class, "0");
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static void operatee(ElevatorState state, double gamepadVal) {
        //if (gamepadVal == 0) {
            switch (state) {
                case INTAKE:
                    leftMotor.setTargetPosition(ElevatorConstants.IntakePos);
                    rightMotor.setTargetPosition(ElevatorConstants.IntakePos);
                    currentElevPos = getElevatorPosL();
                    break;
                case SPECIMEN:
                    leftMotor.setTargetPosition(ElevatorConstants.SpecimenPos);
                    rightMotor.setTargetPosition(ElevatorConstants.SpecimenPos);
                    currentElevPos = getElevatorPosL();
                    break;
            } /*
        }else {
            leftMotor.setTargetPosition((int)(currentElevPos + gamepadVal * 2));
            rightMotor.setTargetPosition((int)(currentElevPos + gamepadVal * 2));
        }
        */


    }


    public static void operate() {
        rightMotor.setTargetPosition(1000);
        leftMotor.setTargetPosition(1000);
    }


    public static void operateTest(int Val) {
        leftMotor.setTargetPosition(Val);

    }

    public static double getElevatorPosL() {
        return leftMotor.getCurrentPosition() - encoderResetValL;
    }

    public static void resetEncoder(){
        encoderResetValR = leftMotor.getCurrentPosition();
        encoderResetValL = leftMotor.getCurrentPosition();
    }
}
