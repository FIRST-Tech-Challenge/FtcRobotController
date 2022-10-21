package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

public class UltraSoundSensor {

    ModernRoboticsI2cRangeSensor RangeSensor;

    public UltraSoundSensor(ModernRoboticsI2cRangeSensor _RangeSensor) {
        RangeSensor = _RangeSensor;
    }

    public double readCmUltra() {
        try {
            return RangeSensor.cmUltrasonic();
        } catch (Exception e) {
            System.out.println("-> Error (cm): " + e.toString());
        }

        return 0;
    }

    public double readRawUltra() {
        try {
            return RangeSensor.rawUltrasonic();
        } catch (Exception e) {
            System.out.println("-> Error (raw): " + e.toString());
        }

        return 0;
    }

    public boolean checkCmUltra(double dist) {
        try {
            double cm = readCmUltra();
//            if (RangeSensor. <= dist) {
//                return true;
//            } else return false;
        } catch (Exception e) {
            System.out.println("-> Error (checkCm): " + e.toString());
        }

        return false;
    }

}
