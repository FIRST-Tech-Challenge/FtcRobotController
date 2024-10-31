package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motors {

    public DcMotor[] driveTrainMotors;

    private final double wheelDiameter = 1.0;
    private final double PPR = 537.7; //how many times the motor counts before a single 360 rotation happens pulse per rotation

//    double leftFrontPower;
//    double rightFrontPower;
//    double leftBackPower;
//    double rightBackPower;


    public Motors(HardwareMap hardwareMap) {

        driveTrainMotors = new DcMotor[4];

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

    public double GetDistance(int motorNumber)
    {
        double CPR = PPR * 4; //count per rotation
        double position = driveTrainMotors[motorNumber].getCurrentPosition();

        double revolutions = position/CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        double circumference = Math.PI * wheelDiameter;

        return circumference * revolutions; // return distance
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
