package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motors {


    enum Type {
        LeftBack (0),
        LeftFront (1),
        RightFront (2),
        RightBack (3),
        Arm (4),
        Claw (5);

        private int value;

        Type(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public DcMotor[] motors;

    private final double wheelDiameter = 1.0;
    private final double PPR = 537.7; //how many times the motor counts before a single 360 rotation happens pulse per rotation

//    double leftFrontPower;
//    double rightFrontPower;
//    double leftBackPower;
//    double rightBackPower;


    public Motors(HardwareMap hardwareMap) {

        motors = new DcMotor[6];
        //i may or may not have subtracted one from motors length cuz instead of 4 3? it'll work i swear
        // it did not work

        motors[Type.LeftBack.getValue()] = hardwareMap.get(DcMotor.class, "0");
        motors[Type.LeftFront.getValue()] = hardwareMap.get(DcMotor.class, "1");
        motors[Type.RightFront.getValue()] = hardwareMap.get(DcMotor.class, "2");
        motors[Type.RightBack.getValue()] = hardwareMap.get(DcMotor.class, "3");
        motors[Type.Arm.getValue()] = hardwareMap.get(DcMotor.class, "4");
        motors[Type.Claw.getValue()] = hardwareMap.get(DcMotor.class, "5");


        motors[Type.LeftBack.getValue()].setDirection(DcMotor.Direction.REVERSE);
        motors[Type.LeftFront.getValue()].setDirection(DcMotor.Direction.REVERSE);
        motors[Type.Arm.getValue()].setDirection(DcMotor.Direction.REVERSE);

        motors[Type.RightFront.getValue()].setDirection(DcMotor.Direction.FORWARD);
        motors[Type.RightBack.getValue()].setDirection(DcMotor.Direction.FORWARD);
        motors[Type.Claw.getValue()].setDirection(DcMotor.Direction.FORWARD);


        motors[Type.Claw.getValue()].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[Type.Claw.getValue()].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors[Type.Arm.getValue()].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



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

    public void MoveMotor(Type motorNumber, double power) { //choose motor to move, power is 0-100, motorNumber is 0-3

        double actualPower = power / 100;

        motors[motorNumber.getValue()].setPower(actualPower);
    }

    public double GetArmDistance()
    {
        double CPR = PPR * 4; //count per rotation
        double position = motors[4].getCurrentPosition();

        double revolutions = position/CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        double circumference = Math.PI * wheelDiameter;

        return circumference * revolutions; // return distance
    }


    public void setArmPosition(int position)
    {
        motors[4].setTargetPosition(position);
    }

    public int getArmPosition()
    {
        return motors[4].getCurrentPosition();
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
