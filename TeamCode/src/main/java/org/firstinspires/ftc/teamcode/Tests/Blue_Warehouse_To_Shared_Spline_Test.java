package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "Blue_Warehouse_To_Shared_Spline_Test", preselectTeleOp = "TwoGPTeleop")
public class Blue_Warehouse_To_Shared_Spline_Test extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //int postion = robot.ElemTest(this, 2, 0);

        robot.setPosition(0,0,0);
        waitForStart();

        robot.autonomousBarrierPathWarehouseToShared(-26.25,1.5,0.5);

        stop();
    }
}