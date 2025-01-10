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
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoMock extends ServoComponent {

    Servo.Direction     mDirection;
    double              mPosition;
    double              mMin;
    double              mMax;

    /* -------------- Constructors --------------- */
    public ServoMock(String name)
    {
        mName = name;
        mDirection = Servo.Direction.FORWARD;
    }

    /* ---------- Servo methods override --------- */

    @Override
    public Servo.Direction	            getDirection()  { return mDirection;  }

    @Override
    public double	                    getPosition()   { return mPosition;   }

    @Override
    public void	                        scaleRange(double min, double max)
    {
        mMin = min;
        mMax = max;
    }

    @Override
    public void	                        setDirection(Servo.Direction direction) { mDirection = direction; }

    @Override
    public void	                        setPosition(double position)
    {
        mPosition = min(position,mMax);
        mPosition = max(mPosition,mMin);
    }

}
