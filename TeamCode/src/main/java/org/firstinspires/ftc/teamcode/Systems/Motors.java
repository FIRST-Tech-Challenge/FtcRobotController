package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motors {

    public DcMotor[] driveTrainMotors;

//    private final double wheelDiameter = 1.0;
//    private final double PPR = 537.7; //how many times the motor counts before a single 360 rotation happens pulse per rotation

//    double leftFrontPower;
//    double rightFrontPower;
//    double leftBackPower;
//    double rightBackPower;


    public Motors(HardwareMap hardwareMap) {

        driveTrainMotors = new DcMotor[4];

        for (int i = 0; i <driveTrainMotors.length; i++) {
            driveTrainMotors[i] = hardwareMap.get(DcMotor.class, Integer.toString(i));

            if(i == 0 || i == 1) {
                driveTrainMotors[i].setDirection(DcMotor.Direction.REVERSE);
            }
            else {
                driveTrainMotors[i].setDirection(DcMotor.Direction.FORWARD);
            }

        }
/*
            This might be a bit difficult to understand at first but the block of code above basically does what the commented out one does below.


            So: the robot looks something like this with "@" representing each on of the 4 wheels and "^" representing which way the robot is looking
            In this above, I have named each wheel with a number

                                    __________
                                1 @ |   ^^   | @ 2
                                    |        |
                                    |        |
                                0 @ |________| @ 3


        It starts at the back left wheel and goes around in a counter clockwise loop until it reaches the back right wheel
        And the arm being the 4th motor

        I made the motor code this way because although it is a bit less readable with the amount of motors we have
        when you use motor.MoveMotor(), you put in a number which can by a number from 0 to 4
        so you wouldn't be able to see in the code what motor you're using

*/





//        driveTrainMotors[0]  = hardwareMap.get(DcMotor.class, "LB"); //left back
//        driveTrainMotors[1]  = hardwareMap.get(DcMotor.class, "LF"); //left front
//        driveTrainMotors[2] = hardwareMap.get(DcMotor.class, "RF"); //right front
//        driveTrainMotors[3] = hardwareMap.get(DcMotor.class, "RB"); //right back
//        driveTrainMotors[4] = hardwareMap.get(DcMotor.class, "ARM"); //right back
//
//        driveTrainMotors[0].setDirection(DcMotor.Direction.REVERSE);
//        driveTrainMotors[1].setDirection(DcMotor.Direction.REVERSE);
//        driveTrainMotors[2].setDirection(DcMotor.Direction.FORWARD);
//        driveTrainMotors[3].setDirection(DcMotor.Direction.FORWARD);
//        driveTrainMotors[4].setDirection(DcMotor.Direction.FORWARD);

    }

    public void MoveMotor(int motorNumber, double power) { //choose motor to move, power is 0-100, motorNumber is 0-3

        double actualPower = power / 100;

        driveTrainMotors[motorNumber].setPower(actualPower);
    }

//    public double GetDistance(int motorNumber)
//    {
//        double CPR = PPR * 4; //count per rotation
//        double position = driveTrainMotors[motorNumber].getCurrentPosition();
//
//        double revolutions = position/CPR;
//
//        double angle = revolutions * 360;
//        double angleNormalized = angle % 360;
//
//        double circumference = Math.PI * wheelDiameter;
//
//        return circumference * revolutions; // return distance
//    }

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
