package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Motors {

    private DcMotor[] driveTrainMotors;

    public void Initialize() {

        driveTrainMotors[0]  = hardwareMap.get(DcMotor.class, "LB"); //left back
        driveTrainMotors[1]  = hardwareMap.get(DcMotor.class, "LF"); //left front
        driveTrainMotors[2] = hardwareMap.get(DcMotor.class, "RB"); //rightBack
        driveTrainMotors[3] = hardwareMap.get(DcMotor.class, "RF"); //rightFront

        driveTrainMotors[0].setDirection(DcMotor.Direction.REVERSE);
        driveTrainMotors[1].setDirection(DcMotor.Direction.REVERSE);
        driveTrainMotors[2].setDirection(DcMotor.Direction.FORWARD);
        driveTrainMotors[3].setDirection(DcMotor.Direction.FORWARD);
    }

    public void MoveMotor(double power, int motorNumber) { //power is 0-100, motorNumber is 0-3

        double actualPower = power / 100;

        driveTrainMotors[motorNumber].setPower(actualPower);
    }

}
