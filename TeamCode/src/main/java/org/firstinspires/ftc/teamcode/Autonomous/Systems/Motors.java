package org.firstinspires.ftc.teamcode.Autonomous.Systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Motors {

    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;

    public void InitializeActuators()
    {
        leftBack  = hardwareMap.get(DcMotor.class, "LB");
        leftFront  = hardwareMap.get(DcMotor.class, "LF");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        rightFront = hardwareMap.get(DcMotor.class, "RF");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
    }

    public void MoveActuators(double power)
    {
        double actualPower = power / 100;

        leftBack.setPower(actualPower);
        leftFront.setPower(actualPower);
        rightBack.setPower(actualPower);
        rightFront.setPower(actualPower);

    }

}
