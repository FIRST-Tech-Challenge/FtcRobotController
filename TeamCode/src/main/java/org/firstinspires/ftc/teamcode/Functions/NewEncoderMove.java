package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class NewEncoderMove {
    private DcMotor leftMotor, leftMotorBack, rightMotor, rightMotorBack;
    private int leftFrontPos, rightFrontPos, leftBackPos, rightBackPos;

    /**
     * This method sets the mode of the motors to run with encoders and resets them
     * It also sets the original position of all motors to 0 so it means it hasn't moved yet
     */
    void Init()
    {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontPos = 0;
        leftBackPos = 0;
        rightFrontPos = 0;
        rightBackPos = 0;

    }
    /**
     * This method initialises the motors.
     * @param _LM : left motor front
     * @param _LMB : right motor front
     * @param _RM : left motor back
     * @param _RMB : right motor back
     */
    public NewEncoderMove(DcMotor _LM, DcMotor _LMB, DcMotor _RM, DcMotor _RMB)
    {
        leftMotor = _LM;
        leftMotorBack = _LMB;
        rightMotor = _RM;
        rightMotorBack = _RMB;
        Init();
    }

    /**
     * This function will be used in our main program to tell the robot to go a certain number of ticks
     * It can also be used for rotating the robot
     * @param leftFrontTarget : the target ticks number for the left front motor
     * @param leftBackTarget : the target ticks number for the left back motor
     * @param rightFrontTarget : the target ticks number for the right front motor
     * @param rightBackTarget : the target ticks number for the right back motor
     * @param power : the power/speed that we set the motors to run with
     * @param opMode : this boolean tells us if the opMode is active or not so that motor will continue to run and go to the target position only if the opMode is active and the code is running
     * Template: DriveTo(+/-number, +/-number, +/-number, +/-number, any number from 0 to 1,  opModeIsActive());
     */
    public void DriveTo(int leftFrontTarget, int leftBackTarget, int rightFrontTarget, int rightBackTarget, double power, boolean opMode)
    {
        leftFrontPos = 0;
        leftBackPos = 0;
        rightFrontPos = 0;
        rightBackPos = 0;

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
