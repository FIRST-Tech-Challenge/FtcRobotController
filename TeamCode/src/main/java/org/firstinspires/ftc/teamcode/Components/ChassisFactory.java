package org.firstinspires.ftc.teamcode.Components;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ChassisFactory {
    public static BasicChassis getChassis(BasicChassis.ChassisType chassisType, LinearOpMode op, boolean navigator){
         if(chassisType==BasicChassis.ChassisType.ENCODER){
            return new EncoderChassis(op);
        }
        else if(chassisType==BasicChassis.ChassisType.IMU){
            return new IMUChassis(op);
        }
        else if(chassisType==BasicChassis.ChassisType.ODOMETRY){
            return new OdometryChassis(op,navigator);
        }
        return null;
    }
}
