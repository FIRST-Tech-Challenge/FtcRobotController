package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmEncoder {
    /**
     * Made by David
     */
    private DcMotor armMotor;
    private ElapsedTime runtime = new ElapsedTime();
    public ArmEncoder(DcMotor _AM)
    {
        armMotor = _AM;
        Init();
    }
    public void Init()
    {
        /**
         * Runs the motor using encoders.
         */
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void goTo(int ticks)
    {
        /**
         * Motor is stopped and the encoder value is reset to 0.
         */
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /**
         * Sets the ticks target of the motors.
         */
        armMotor.setTargetPosition(ticks);

        /**
         * Runs to the ticks value that we want.
         */
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /**
         * Sets the velocity of the motors.
         */
//        armMotor.setVelocity(velocity);
        /**
         * This while is/can be empty because we need to run the function until the motors have reached the target
         */


    }
}