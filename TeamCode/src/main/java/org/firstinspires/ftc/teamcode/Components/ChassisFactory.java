package org.firstinspires.ftc.teamcode.Components;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ChassisFactory {
    public static BasicChassis getChassis(BasicChassis.ChassisType chassisType, LinearOpMode op, boolean navigator, boolean isCorgi, Logger logger){
         if(chassisType==BasicChassis.ChassisType.ENCODER){
            return new EncoderChassis(op, navigator, isCorgi, logger);
        }
        else if(chassisType==BasicChassis.ChassisType.IMU){
            return new IMUChassis(op);
        }
        else if(chassisType==BasicChassis.ChassisType.ODOMETRY){
            return new OdometryChassis(op,navigator,isCorgi);
        }
         else if(chassisType==BasicChassis.ChassisType.VSLAM){
             return new VSLAMChassis(op,navigator,isCorgi);
         }
        return null;
    }
}
