/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   CoupledServo class overloads the FTC servo class to manage
   A couple of servos both turning the same hardware.

   Note that this is a dangerous situation which can result in
   servo destruction if not correctly tuned. The coupled servos
   shall be tuned so that each orientation of the hardware they
   both support correspond to the same position on the 2 servos.
   If wrongly tuned, each of the 2 coupled servos may end up
   each forcing into a position they can not reach without the
   other failing.

   This means for example that the 2 servos are the same model
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.components;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorMock extends MotorComponent {

    DcMotorSimple.Direction     mDirection;
    DcMotor.RunMode             mMode;
    int                         mPosition;
    DcMotor.ZeroPowerBehavior   mBehavior;
    double                      mPower;

    public MotorMock(String name)
    {
        mName = name;
        mDirection = DcMotorSimple.Direction.FORWARD;
        mMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        mBehavior = DcMotor.ZeroPowerBehavior.UNKNOWN;
    }

    @Override
    public int	                        getCurrentPosition() { return mPosition; }

    @Override
    public DcMotorSimple.Direction      getDirection()
    {
        return mDirection;
    }

    @Override
    public DcMotor.RunMode	            getMode() { return mMode; }

    @Override
    public int	                        getTargetPosition() { return mPosition; }

    @Override
    public DcMotor.ZeroPowerBehavior	getZeroPowerBehavior() { return mBehavior; }

    @Override
    public double                   	getPower() { return mPower; }

    @Override
    public boolean	                    isBusy() { return false; }

    @Override
    public void	                        setMode(DcMotor.RunMode mode) { mMode = mode; }

    @Override
    public void	                        setDirection(DcMotorSimple.Direction direction) { mDirection = direction; }

    @Override
    public void	                        setTargetPosition(int position) { mPosition = position;}

    @Override
    public void	                        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) { mBehavior = zeroPowerBehavior; }

    @Override
    public void	                        setPower(double power) { mPower = power; }

}
