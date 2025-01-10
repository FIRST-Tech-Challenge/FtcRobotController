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

/* System includes */
import java.util.ArrayList;
import java.util.ListIterator;
import java.util.Map;
import java.util.List;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;

public class MotorCoupled extends MotorComponent {

    Telemetry                       mLogger;

    DcMotorSimple.Direction         mDirection;

    DcMotor                         mFirst;
    DcMotor                         mSecond;

    /* -------------- Constructors --------------- */
    public MotorCoupled(ConfMotor conf, HardwareMap hwMap, String name, Telemetry logger)
    {
        mReady  = true;
        mLogger = logger;
        mName   = name;
        mDirection = DcMotor.Direction.FORWARD;

        Map<String, Boolean> hw = conf.getHw();
        if((hw.size() == 2) && !conf.shallMock()) {

            List<Map.Entry<String, Boolean>> motors = new ArrayList<>(hw.entrySet());
            ListIterator<Map.Entry<String, Boolean>> iterator = motors.listIterator();

            Map.Entry<String,Boolean> motor = iterator.next();
            mFirst = hwMap.tryGet(DcMotor.class, motor.getKey());
            if(mFirst != null && motor.getValue()) { mFirst.setDirection(DcMotor.Direction.REVERSE);}
            else if(mFirst != null)                { mFirst.setDirection(DcMotor.Direction.FORWARD);}

            motor = iterator.next();
            mSecond = hwMap.tryGet(DcMotor.class, motor.getKey());
            if(mSecond != null && motor.getValue()) { mSecond.setDirection(DcMotor.Direction.REVERSE);}
            else if(mSecond != null)                { mSecond.setDirection(DcMotor.Direction.FORWARD);}
        }

        if(mFirst  == null) { mReady = false; }
        if(mSecond == null) { mReady = false; }
    }

    @Override
    public int	                        getCurrentPosition()
    {
        int result = -1;
        if(mReady) {
            result = (int) (0.5 * mFirst.getCurrentPosition() + 0.5 * mSecond.getCurrentPosition());
        }
        return result;
    }

    @Override
    public DcMotorSimple.Direction      getDirection()
    {
        return mDirection;
    }


    @Override
    public DcMotor.RunMode	            getMode()
    {
        DcMotor.RunMode result =  DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        if (mReady) { result = mFirst.getMode(); }
        return result;
    }

    @Override
    public int	                        getTargetPosition()
    {
        int result = -1;
        if(mReady) {
            result = (int) (0.5 * mFirst.getTargetPosition() + 0.5 * mSecond.getTargetPosition());
        }
        return result;
    }

    @Override
    public double	                    getPower()
    {
        double result = -1;
        if(mReady) {
            result = (int) (0.5 * mFirst.getPower() + 0.5 * mSecond.getPower());
        }
        return result;
    }

    @Override
    public DcMotor.ZeroPowerBehavior	getZeroPowerBehavior()
    {
        DcMotor.ZeroPowerBehavior result = DcMotor.ZeroPowerBehavior.UNKNOWN;
        if(mReady) { result = mFirst.getZeroPowerBehavior(); }
        return result;
    }

    @Override
    public boolean	                    isBusy()
    {
        boolean result = false;
        if(mReady) { result = (mFirst.isBusy() || mSecond.isBusy()); }
        return result;
    }

    @Override
    public void	                        setMode(DcMotor.RunMode mode)
    {
        if(mReady) {
            mFirst.setMode(mode);
            mSecond.setMode(mode);
        }
    }

    @Override
    public void	                        setDirection(DcMotorSimple.Direction direction)
    {
        if(direction != mDirection && mReady) {

            if(     mFirst.getDirection()  == DcMotor.Direction.FORWARD) { mFirst.setDirection(DcMotor.Direction.REVERSE);  }
            else if(mFirst.getDirection()  == DcMotor.Direction.REVERSE) { mFirst.setDirection(DcMotor.Direction.FORWARD);  }

            if(     mSecond.getDirection() == DcMotor.Direction.FORWARD) { mSecond.setDirection(DcMotor.Direction.REVERSE); }
            else if(mSecond.getDirection() == DcMotor.Direction.REVERSE) { mSecond.setDirection(DcMotor.Direction.FORWARD); }

            mDirection = direction;

        }
    }

    @Override
    public void	                        setTargetPosition(int position)
    {
        if(mReady) {
            mFirst.setTargetPosition(position);
            mSecond.setTargetPosition(position);
        }
    }

    @Override
    public void	                        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        if(mReady) {
            mFirst.setZeroPowerBehavior(zeroPowerBehavior);
            mSecond.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    @Override
    public void	                        setPower(double power)
    {
        if(mReady) {
            mFirst.setPower(power);
            mSecond.setPower(power);
        }
    }

}
