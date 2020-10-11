package org.darbots.darbotsftclib.libcore.runtime;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.darbotsftclib.libcore.templates.servo_related.ContinuousRotationServoType;
import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorUtil {
    public static void setServoPulseWidth(Servo servo, ServoType servoType){
        if(servo instanceof PwmControl){
            PwmControl servoPWM = (PwmControl) servo;
            servoPWM.setPwmRange(
                    new PwmControl.PwmRange(
                        servoType.getPulseLowerInMicroSeconds(),
                        servoType.getPulseUpperInMicroSeconds()
                    )
            );
        }
    }
    public static void setCRServoPulseWidth(CRServo crServo, ContinuousRotationServoType CRServoType){
        if(crServo instanceof PwmControl){
            PwmControl CRServoPWM = (PwmControl) crServo;
            CRServoPWM.setPwmRange(
                    new PwmControl.PwmRange(
                            CRServoType.getPulseLowerInMicroSeconds(),
                            CRServoType.getPulseUpperInMicroSeconds()
                    )
            );
        }
    }
    public static double getDistanceSensorReading(DistanceSensor sensor, int trial){
        int successfulTrials = 0;
        double distSum = 0;
        for(int i = 0; i < trial; i++){
            double reading = sensor.getDistance(DistanceUnit.CM);
            if(reading != DistanceSensor.distanceOutOfRange){
                distSum += reading;
                successfulTrials++;
            }
        }
        if(successfulTrials == 0){
            return DistanceSensor.distanceOutOfRange;
        }else{
            return distSum / successfulTrials;
        }
    }
}
