package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motors {


    public enum Type {
        LeftBack (0),
        LeftFront (1),
        RightFront (2),
        RightBack (3),
        Arm (4), // the arm that swings back and forth
        Pull(5); // the arm that goes up and down

        private final int value;

        Type(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public DcMotor[] motors;


    public Motors(HardwareMap hardwareMap) {

        motors = new DcMotor[Type.values().length];

        motors[Type.LeftBack.getValue()] = hardwareMap.get(DcMotor.class, "0");
        motors[Type.LeftFront.getValue()] = hardwareMap.get(DcMotor.class, "1");
        motors[Type.RightFront.getValue()] = hardwareMap.get(DcMotor.class, "2");
        motors[Type.RightBack.getValue()] = hardwareMap.get(DcMotor.class, "3");
        motors[Type.Arm.getValue()] = hardwareMap.get(DcMotor.class, "4");
        motors[Type.Pull.getValue()] = hardwareMap.get(DcMotor.class, "5");


        motors[Type.LeftBack.getValue()].setDirection(DcMotor.Direction.REVERSE);
        motors[Type.LeftFront.getValue()].setDirection(DcMotor.Direction.REVERSE);
        motors[Type.Arm.getValue()].setDirection(DcMotor.Direction.REVERSE);

        motors[Type.RightFront.getValue()].setDirection(DcMotor.Direction.FORWARD);
        motors[Type.RightBack.getValue()].setDirection(DcMotor.Direction.FORWARD);
        motors[Type.Pull.getValue()].setDirection(DcMotor.Direction.FORWARD);


        motors[Type.Arm.getValue()].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // doesn't actually stop the motor from moving, just slows it down so it doesn't slam into the ground

    }

    public void MoveMotor(Type motorNumber, double power) { //choose motor to move with type and move with power is 0-100

        double actualPower = power / 100;

        motors[motorNumber.getValue()].setPower(actualPower);
    }
    public int getArmPosition()
    {
        return motors[Type.Arm.getValue()].getCurrentPosition();
    }

    public int getUpArmPosition()
    {
        return motors[Type.Pull.getValue()].getCurrentPosition();
    }

}
