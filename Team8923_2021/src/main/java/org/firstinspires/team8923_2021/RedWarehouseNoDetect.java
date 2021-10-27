package org.firstinspires.team8923_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedWarehouseNoDetect")
abstract public class RedWarehouseNoDetect extends MasterAutonomous{


    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        while (opModeIsActive()){
            moveAuto(0, 5, 20, 10);
            break;
        }
    }





}

