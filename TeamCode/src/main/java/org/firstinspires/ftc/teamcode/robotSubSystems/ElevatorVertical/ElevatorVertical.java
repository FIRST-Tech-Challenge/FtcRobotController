package org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.PID;

public class ElevatorVertical {
    public static DcMotor leftMotor;
    public static DcMotor rightMotor;

    private static int wantedPos = 0;

    private static final PID changeLevelPID = new PID(
            ElevatorVerticalConstants.changeLevelKp,
            ElevatorVerticalConstants.changeLevelKi,
            ElevatorVerticalConstants.changeLevelKd,
            ElevatorVerticalConstants.changeLevelKf,
            ElevatorVerticalConstants.changeLevelIzone,
            ElevatorVerticalConstants.changeLevelMaxSpeed,
            ElevatorVerticalConstants.changeLevelMinSpeed
    );


    public static void init(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotor.class, "1");
        leftMotor = hardwareMap.get(DcMotor.class, "0");
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        resetEncoder();
    }
    private static double power = 0;
    private static ElevatorVerticalState lastWantedState = ElevatorVerticalState.INTAKE;
    public static void operate(ElevatorVerticalState wantedState, double gamepadVal, Telemetry telemetry) {
            if (gamepadVal == 0 && wantedState != lastWantedState) {
                switch (wantedState) {
                    case INTAKE:
                        wantedPos = ElevatorVerticalConstants.IntakePos;
                        break;
                    case SPECIMEN:
                        wantedPos = ElevatorVerticalConstants.SpecimenPos;
                        break;
                    case PUTSPECIMEN:
                        wantedPos = ElevatorVerticalConstants.PutSpecimenPos;
                        break;
                    case DEPLETE:
                        wantedPos = ElevatorVerticalConstants.DepletePos;
                        break;
                }
                lastWantedState = wantedState;
            }else {
                wantedPos += (int) gamepadVal * 35;
            }
            /*
            if (wantedState != ElevatorVerticalState.INTAKE) {
                wantedPos += (int) gamepadVal * 25;
            }
             */
//            if(wantedPos > 2235) { wantedPos = 2235; } else if(wantedPos < 0) { wantedPos = 0; }
            //wantedPos = limiter(wantedPos, 0, ElevatorVerticalConstants.MAX_OPEN_POS);
            changeLevelPID.setWanted(wantedPos);
            power = changeLevelPID.update(getElevatorPos());

            leftMotor.setPower(power);
            rightMotor.setPower(power);
    }
    private static int encoderResetVal = 0;
    private static int encoderResetValL = 0;
    public static int getElevatorPos() {
        return leftMotor.getCurrentPosition() - encoderResetVal;
    }

    public static int getWantedPos(){return wantedPos;}

    public static int limiter(int pose , int lowVal, int highVal) {
        if(pose > highVal) { pose = highVal; } else if(pose < lowVal) { pose = lowVal; }
        return pose;
    }

    public static void resetEncoder(){
        encoderResetVal = leftMotor.getCurrentPosition();
        encoderResetValL = leftMotor.getCurrentPosition();
    }

    private static void setFloor(int wantedPos){

        changeLevelPID.setWanted(wantedPos);

        leftMotor.setPower(changeLevelPID.update(leftMotor.getCurrentPosition()));
        rightMotor.setPower(changeLevelPID.update(leftMotor.getCurrentPosition()));
    }
    public static void test() {
        rightMotor.setPower(0.5);
    }
}

/*
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
                rightMotor.setPower(1);
                leftMotor.setPower(1);
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentElevPosL = getElevatorPosL();
                currentElevPosR = getElevatorPosR();
                break;


            case SPECIMEN:
                leftMotor.setTargetPosition(ElevatorConstants.SpecimenPos);
                rightMotor.setTargetPosition(ElevatorConstants.SpecimenPos);
                //power is elevator speed
                rightMotor.setPower(1);
                leftMotor.setPower(1);
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentElevPosL = getElevatorPosL();
                currentElevPosR = getElevatorPosR();
                break;

            case PUTSPECIMEN:
                leftMotor.setTargetPosition(ElevatorConstants.PutSpecimenPos);
                rightMotor.setTargetPosition(ElevatorConstants.PutSpecimenPos);
                //power is elevator speed
                rightMotor.setPower(1);
                leftMotor.setPower(1);
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentElevPosL = getElevatorPosL();
                currentElevPosR = getElevatorPosR();
                break;
            case MANUAL:
                //the number is the elevator speed (PID already in the settargetposition func)
                leftMotor.setTargetPosition((int) (currentElevPosL + gamepadVal * 220));
                rightMotor.setTargetPosition((int) (currentElevPosR + gamepadVal * 220));
                rightMotor.setPower(1);
                leftMotor.setPower(1);
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentElevPosL = getElevatorPosL();
                currentElevPosR = getElevatorPosR();
                break;
        }
    }

    public static void operate(double val) {
        rightMotor.setPower(val);
        leftMotor.setPower(val);
    }

    public static double getCurrentElevPosL() {
        return currentElevPosL;
    }
    public static void resetElev() {
        currentElevPosL = 0;
        currentElevPosR = 0;
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

 */
