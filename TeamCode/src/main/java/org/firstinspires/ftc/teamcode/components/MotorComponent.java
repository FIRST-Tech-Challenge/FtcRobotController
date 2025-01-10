/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   CoupledMotor class overloads the FTC motor class to manage
   A couple of motors both turning the same hardware.

   Note that this is a dangerous situation which can result in
   motor destruction if not correctly tuned. The coupled motors
   shall be the same model
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.components;


/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class MotorComponent {

    protected boolean  mReady;
    protected String   mName;

    public boolean                              isReady() { return mReady;}
    public String                               getName() { return mName; }
    public abstract boolean	                    isBusy();

    public abstract int	                        getCurrentPosition();
    public abstract DcMotor.RunMode	            getMode();
    public abstract int	                        getTargetPosition();
    public abstract DcMotorSimple.Direction     getDirection();
    public abstract DcMotor.ZeroPowerBehavior	getZeroPowerBehavior();
    public abstract double                      getPower();

    public abstract void	                    setMode(DcMotor.RunMode mode);
    public abstract void	                    setDirection(DcMotorSimple.Direction direction);
    public abstract void	                    setTargetPosition(int position);
    public abstract void	                    setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior);
    public abstract void                        setPower(double power);

}
