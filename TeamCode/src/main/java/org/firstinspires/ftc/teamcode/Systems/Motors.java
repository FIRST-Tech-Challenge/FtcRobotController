package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Motors {

    public DcMotor[] driveTrainMotors = null;

    private final double wheelDiameter = 1.0;
    //private final double CPR = ; //how many times the motor counts before a single 360 rotation happens
    private final double circumference = Math.PI * wheelDiameter;

//    double leftFrontPower;
//    double rightFrontPower;
//    double leftBackPower;
//    double rightBackPower;


    public void Initialize() {

        driveTrainMotors[0]  = hardwareMap.get(DcMotor.class, "LB"); //left back
        driveTrainMotors[1]  = hardwareMap.get(DcMotor.class, "LF"); //left front
        driveTrainMotors[2] = hardwareMap.get(DcMotor.class, "RF"); //right front
        driveTrainMotors[3] = hardwareMap.get(DcMotor.class, "RB"); //right back

        driveTrainMotors[0].setDirection(DcMotor.Direction.REVERSE);
        driveTrainMotors[1].setDirection(DcMotor.Direction.REVERSE);
        driveTrainMotors[2].setDirection(DcMotor.Direction.FORWARD);
        driveTrainMotors[3].setDirection(DcMotor.Direction.FORWARD);

        driveTrainMotors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrainMotors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrainMotors[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrainMotors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void MoveMotor(int motorNumber, double power) { //choose motor to move, power is 0-100, motorNumber is 0-3

        double actualPower = power / 100;

        driveTrainMotors[motorNumber].setPower(actualPower);
    }

    public double GetMotorPosition(int motorNumber)
    {
        return driveTrainMotors[motorNumber].getCurrentPosition();
    }

//    private void CalculatePower()
//    {
//        double max;
//
//        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower  /= max;
//            rightFrontPower /= max;
//            leftBackPower   /= max;
//            rightBackPower  /= max;
//        }
//    }

}
