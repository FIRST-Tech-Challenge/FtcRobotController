package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;

public class NewEncoderMove {
    private DcMotor leftMotor, leftMotorBack, rightMotor, rightMotorBack;
    private int leftFrontPos, rightFrontPos, leftBackPos, rightBackPos;

    void Init()
    {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontPos = 0;
        leftBackPos = 0;
        rightFrontPos = 0;
        rightBackPos = 0;

    }

    public NewEncoderMove(DcMotor _LM, DcMotor _LMB, DcMotor _RM, DcMotor _RMB)
    {
        leftMotor = _LM;
        leftMotorBack = _LMB;
        rightMotor = _RM;
        rightMotorBack = _RMB;
        Init();
    }

    public void DriveTo(int leftFrontTarget, int leftBackTarget, int rightFrontTarget, int rightBackTarget, double power, boolean opMode)
    {
        leftFrontPos += leftFrontTarget;
        leftBackPos += leftBackTarget;
        rightFrontPos += rightFrontTarget;
        rightBackPos += rightBackTarget;

        leftMotor.setTargetPosition(leftFrontPos);
        leftMotorBack.setTargetPosition(leftBackPos);
        rightMotor.setTargetPosition(rightFrontPos);
        rightMotorBack.setTargetPosition(rightBackPos);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(power);
        leftMotorBack.setPower(power);
        rightMotor.setPower(power);
        rightMotorBack.setPower(power);

        while(opMode && leftMotor.isBusy() && leftMotorBack.isBusy() && rightMotor.isBusy() && rightMotorBack.isBusy())
        {

        }
    }
}
