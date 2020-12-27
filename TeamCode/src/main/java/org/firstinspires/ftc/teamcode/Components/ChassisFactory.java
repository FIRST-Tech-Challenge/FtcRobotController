package org.firstinspires.ftc.teamcode.Components;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.Chassis;
import org.firstinspires.ftc.teamcode.Components.EncoderChassis;
import org.firstinspires.ftc.teamcode.Components.IMUChassis;

public class ChassisFactory {
    public static BasicChassis getChassis(BasicChassis.ChassisType chassisType, LinearOpMode op){
         if(chassisType==BasicChassis.ChassisType.ENCODER){
            return new EncoderChassis(op);
        }
        else if(chassisType==BasicChassis.ChassisType.IMU){
            return new IMUChassis(op);
        }
        else if(chassisType==BasicChassis.ChassisType.ODOMETRY){
            return new OdometryChassis(op);
        }
        return null;
    }
}
