package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmEncoder {
    /**
     * Made by David
     */
    private DcMotorEx armMotorLeft, armMotorRight;
    private ElapsedTime runtime = new ElapsedTime();
    public ArmEncoder(DcMotorEx _AML, DcMotorEx _AMR)
    {
        armMotorLeft = _AML;
        armMotorRight = _AMR;
        Init();
    }
    public void Init()
    {
        /**
         * Runs the motor using encoders.
         */
        armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void goTo(int ticks, double velocity)
    {
        /**
         * Motor is stopped and the encoder value is reset to 0.
         */
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /**
         * Sets the ticks target of the motors.
         */
        armMotorLeft.setTargetPosition(ticks);
        armMotorRight.setTargetPosition(ticks);
        /**
         * Runs to the ticks value that we want.
         */
        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /**
         * Sets the velocity of the motors.
         */
        armMotorLeft.setVelocity(velocity);
        armMotorRight.setVelocity(velocity);
        /**
         * This while is/can be empty because we need to run the function until the motors have reached the target
         */
//        while(armMotorLeft.isBusy() && armMotorRight.isBusy())
//        {
//
//        }


    }
}