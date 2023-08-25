package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Class for two 360deg-rotation potentiometers, that measure absolute angle of
 * the wheel angles of a two wheeled swerve drive robot.
 * 306-potentiometers are two pots in one, that are out of phase by about 90 degrees.
 * They have two signals, thus they take two analog ports each.
 * NOTE: The REV HUB Analog Input Ports give NON-LINEAR results.
 * The voltage on the input ports needs to be converted through a function to get angles.
 */
public class TwoFullRotationPotClass {
    // CONSTANTS
    static final int SAMPLES = 2;  // number of readings averaged for one reading

    // Members
    public double angle1; // angle of wheel 1
    public double angle2; // angle of wheel 2

    // Potentiometers analog voltage values from REV HUB
    private AnalogInput potentiometer1;
    private AnalogInput potentiometer2;
    private AnalogInput potentiometer3;
    private AnalogInput potentiometer4;

    // Constructor
    public TwoFullRotationPotClass() {
        this.angle1 = 0.0;
        this.angle2 = 0.0;
    }

    /**
     * Initialize Potentiometers. initPots must be called from init()
     * @param hwMap Pass the Hardware map object from the OpMode
     */
    public void initPots(HardwareMap hwMap) {
        // map potentiometers
        potentiometer1 = hwMap.get(AnalogInput.class,"n1");
        potentiometer2 = hwMap.get(AnalogInput.class,"n2");
        potentiometer3 = hwMap.get(AnalogInput.class,"n3");
        potentiometer4 = hwMap.get(AnalogInput.class,"n4");
    }

    /**
     * Does conversion of voltages to angle for Potentiometer 1
     * The constants in the calls to polynomial3 are derived in a spreadsheet
     * @param voltsP1 first analog channel
     * @param voltsP2 second analog channel
     * @param low lower cutoff for voltages
     * @param high high cutoff for voltages
     * @return angle of the potentiometer
     */
    public double getAngleFromPot1(double voltsP1, double voltsP2, double low, double high) {
        double angle;
        double average;
        boolean v1good, v2good;
        double deltaV;
        deltaV = Math.abs(voltsP1-voltsP2);

        average = (voltsP1 + voltsP2)/2.0;

        v1good= (voltsP1 >= low) && (voltsP1 < high);

        v2good= (voltsP2 >= low) && (voltsP2 < high);

        // new polynomial data from CW and CCW recording and with weight on wheels
        if(deltaV < 0.1) {
            if (average<0.45) {
                angle = polynomial3(voltsP2,7.3142,-12.344,14.941,-6.1918);
            } else {
                angle = polynomial3(voltsP1,5.5681,-11.219,13.191,-5.4138);
            }
        } else if(v1good && (voltsP1>average)) {
            if(voltsP1>0.32) angle = polynomial3(voltsP1,-1.0087,3.3158,0.8167,-2.493);
            else angle = polynomial3(voltsP1,4.4546,5.0852,1.7295,0.0);
        } else if(v2good && (voltsP2<average)) {
            angle = polynomial3(voltsP2,-0.8723,11.917,-14.389,6.0141);
        } else if(v1good && (voltsP1<average)) {
            angle = polynomial3(voltsP1,5.5681,-11.219,13.191,-5.4138);
        } else if(v2good && (voltsP2>average)) {
            angle = polynomial3(voltsP2,7.3142,-12.344,14.941,-6.1918);
        } else {
            angle = 0;
        }
        // Convert current angle into number from zero to 2*PI
        angle = moduloAngle(angle);
        return angle;
    }
    public double getAngleFromPot2(double voltsP1, double voltsP2, double low, double high) {
        double angle;
        double average;
        boolean v1good, v2good;
        double deltaV;
        deltaV = Math.abs(voltsP1-voltsP2);

        average = (voltsP1 + voltsP2)/2.0;

        v1good= (voltsP1 >= low) && (voltsP1 < high);

        v2good= (voltsP2 >= low) && (voltsP2 < high);

        // new polynomial data from CW and CCW recording and with weight on wheels
        if(deltaV < 0.1) {
            if (average < 0.45) {
                angle = polynomial3(voltsP2,7.3369,-12.51,15.278,-6.4954);
            } else {
                angle = polynomial3(voltsP1,5.6767,-11.997,14.628,-6.3041);
            }
        } else if(v1good&&(voltsP1>average)) {
            if(voltsP1>0.31) angle = polynomial3(voltsP1,-1.3312,5.3099,-3.2393,0.0);
            else angle = polynomial3(voltsP1,4.0373,8.974,-6.9517,0.0);
        } else if(v2good&&(voltsP2<average)) {
            angle = polynomial3(voltsP2,-0.964,11.7,-13.903,5.7541);
        } else if(v1good&&(voltsP1<average)) {
            angle = polynomial3(voltsP1,5.6767,-11.997,14.628,-6.3041);
        } else if(v2good&&(voltsP2>average)) {
            angle = polynomial3(voltsP2,7.3369,-12.51,15.278,-6.4954);
        } else {
            angle = 0;
        }
        // Convert current angle into number from zero to 2*PI
        angle = moduloAngle(angle);
        return angle;
    }

    /**
     * Return the two angles from the potentiometers
     * @param log   if true, write to the log file. Use to determine polynomial constants in spreadsheet
      */
    public void getAngleFromPots(boolean log, double logAng) {
        double vPot1,vPot2,vPot3,vPot4;

        vPot1 = 0;
        vPot2 = 0;
        vPot3 = 0;
        vPot4 = 0;
        for(int i=0; i<SAMPLES;i++) {
            vPot1 += potentiometer1.getVoltage();
            vPot2 += potentiometer2.getVoltage();
            vPot3 += potentiometer3.getVoltage();
            vPot4 += potentiometer4.getVoltage();
        }
        vPot1 = vPot1/SAMPLES;
        vPot2 = vPot2/SAMPLES;
        vPot3 = vPot3/SAMPLES;
        vPot4 = vPot4/SAMPLES;

        this.angle1=getAngleFromPot1(vPot1,vPot2,0.17,0.70);
        this.angle2=getAngleFromPot2(vPot3,vPot4,0.17,0.70);

        if(log) {
            RobotLog.d("LOG ANG = %.05f,POT 1 ANGLE (RAD) = %.05f, V1=%.05f,V2=%.05f",logAng,this.angle1,vPot1,vPot2);
            RobotLog.d("LOG ANG = %.05f,POT 2 ANGLE (RAD) = %.05f, V1=%.05f,V2=%.05f",logAng,this.angle2,vPot3,vPot4);
        }
    }

    /**
     * Math routine to solve a 3rd order polynomial
     * @param x for this class, the voltage
     * @param c0 constant c0
     * @param c1 constant c1
     * @param c2 constant c2
     * @param c3 constant c3
     * @return the y value for the x value
     */
    private double polynomial3(double x,double c0,double c1,double c2,double c3) {
        double y;
        y = c0 + c1*x + c2*x*x + c3*x*x*x;
        return y;
    }

    /**
     * Returns an angle from 0 to 2*PI for any real angle
     * @param inAngle any real angle
     * @return an angle from 0 to 2*PI
     */
    public double moduloAngle(double inAngle) {
        double currentRemainderAngle;

        // Convert current angle into number from zero to 2*PI
        currentRemainderAngle = inAngle % (2.0*Math.PI);
        // Java modulo can return negative remainder, so the next step is needed
        if (currentRemainderAngle < 0.0) currentRemainderAngle += 2.0*Math.PI;
        return currentRemainderAngle;
    }

}
