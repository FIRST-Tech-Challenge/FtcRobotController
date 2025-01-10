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

/* System includes */
import java.util.ArrayList;
import java.util.ListIterator;
import java.util.Map;
import java.util.List;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.HardwareDevice;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfServo;

public class ServoSingle extends ServoComponent {

    Telemetry               mLogger;

    Servo.Direction         mDirection;

    Servo                   mServo;

    /* -------------- Constructors --------------- */
    public ServoSingle(ConfServo conf, HardwareMap hwMap, String name, Telemetry logger)
    {
        mReady = true;
        mLogger = logger;
        mDirection = Servo.Direction.FORWARD;
        mName = name;

        Map<String, Boolean> hw = conf.getHw();
        if((hw.size() == 1) && !conf.shallMock()) {

            List<Map.Entry<String, Boolean>> servos = new ArrayList<>(hw.entrySet());
            ListIterator<Map.Entry<String, Boolean>> iterator = servos.listIterator();

            Map.Entry<String,Boolean> servo = iterator.next();
            mServo = hwMap.tryGet(Servo.class, servo.getKey());
            if(mServo != null && servo.getValue()) { mServo.setDirection(Servo.Direction.REVERSE);}
            else if(mServo != null)                { mServo.setDirection(Servo.Direction.FORWARD);}

        }

        if(mServo  == null) { mReady = false; }
    }

    @Override
    public Servo.Direction	            getDirection()
    {
        Servo.Direction result = Servo.Direction.FORWARD;
        if(mReady) {
            result = mServo.getDirection();
        }
        return result;
    }

    @Override
    public double	                    getPosition()
    {
        double result = -1;
        if(mReady) {
            result = mServo.getPosition();
        }
        return result;
    }

    @Override
    public void	                        scaleRange(double min, double max)
    {
        if(mReady) {
            mServo.scaleRange(min, max);
        }
    }

    @Override
    public void	                        setDirection(Servo.Direction direction)
    {
        if(mReady) {
            mServo.setDirection(direction);
        }
    }

    @Override
    public void	                        setPosition(double position)
    {
        if(mReady) {
            mServo.setPosition(position);
        }
    }
}
