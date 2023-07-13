package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis;

public class ChassisFactory {
    public static BasicChassis getChassis(BasicChassis.ChassisType chassisType, boolean navigator, boolean isCorgi){
         if(chassisType==BasicChassis.ChassisType.ENCODER){
            return new EncoderChassis(navigator, isCorgi);
        }
        else if(chassisType==BasicChassis.ChassisType.IMU){
            return new IMUChassis();
        }
        else if(chassisType==BasicChassis.ChassisType.ODOMETRY){
            return new OdometryChassis(navigator,isCorgi);
        }
         else if(chassisType==BasicChassis.ChassisType.VSLAM){
             return new VSLAMChassis(navigator,isCorgi);
         }
        return null;
    }
}
