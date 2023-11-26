package org.firstinspires.ftc.teamcode.utilities;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CASH_Utility_Library {

    public void init(){}

    //Properties to setup sensors/servos
    public ColorSensor floorColorSensor;
    public DistanceSensor fourRingDistanceSensor;
    public DistanceSensor oneRingDistanceSensor;
    public DcMotor liftMotor; //2019-2020 season
    public DcMotor grabArm; //2019-2020 season
    public Servo grab; //2019-2020 season
    public Servo fireServo; //2019-2020 season
    public DcMotor diskGrabberArm; //2019-2020 season
    public Servo diskGrabber; //2019-2020 season
    public DigitalChannel wobbleGoalArmStopSensor;

    public DistanceSensor rightDistSensor;
    public DistanceSensor leftDistSensor;
    //Drop the grabber while the button is down.
    //Lift the grabber when the button is not pressed.
    public double Calculate_Grabber_Power(double right_trigger, double left_trigger){
        if(right_trigger == 1 ){
            return .63;
        }
//        else if (left_trigger == 1) {
//            return .0;
//        }
        else {
            return .3;
        }
    }

    public void LiftPosition(int position) {
        liftMotor.setTargetPosition(position);
    }


    public void GrabArmPosition(boolean up)
    {
        if(up) {
            grabArm.setTargetPosition(-60);
        }else
        {
            grabArm.setTargetPosition(60);
        }
        grabArm.setPower(.2);
    }


    public void Grabber(boolean open) {
        if(open)
        {
            grab.setPosition(0);
        }
        else
        {
            grab.setPosition(.8);
        }
    }

    public float GetColor() {
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        Color.RGBToHSV((int) (floorColorSensor.red() * SCALE_FACTOR),
                (int) (floorColorSensor.green() * SCALE_FACTOR),
                (int) (floorColorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        return hsvValues[0];
    }
    public float[] GetColor2() {
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        float rgbValues[] = {0F, 0F, 0F};

        Color.RGBToHSV((int) (floorColorSensor.red() * SCALE_FACTOR),
                (int) (floorColorSensor.green() * SCALE_FACTOR),
                (int) (floorColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        rgbValues[0]=floorColorSensor.red();
        rgbValues[1]=floorColorSensor.green();
        rgbValues[2]=floorColorSensor.blue();

        return rgbValues;
    }

    public double ReadDistanceSensor(Boolean isFourRingDistanceSensor) {
        if (isFourRingDistanceSensor) {
            return fourRingDistanceSensor.getDistance(DistanceUnit.MM);
        } else {
            return oneRingDistanceSensor.getDistance(DistanceUnit.MM);
        }
    }

    public double ReadFreightDistSensor(boolean isRight) {
        if (isRight) {
            return rightDistSensor.getDistance(DistanceUnit.MM);
        } else {
            return leftDistSensor.getDistance(DistanceUnit.MM);
        }
    }

    public void SweeperPosition(boolean up)
    {
        if(up)
        {
            diskGrabber.setPosition(0);
        }
        else
        {
            diskGrabber.setPosition(.8);
        }

    }
    public void DiskGrabberArmUp (boolean up)
    {
        if (up)
        {
            diskGrabberArm.setTargetPosition(500);
            //diskGrabberArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            diskGrabberArm.setTargetPosition(0);
            //diskGrabberArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        diskGrabberArm.setPower(.7);
    }

}
