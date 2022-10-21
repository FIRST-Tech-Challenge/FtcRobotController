package org.firstinspires.ftc.team8923_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueLeftDetectPark")
public class BlueLeftDetectPark extends BaseAutonomous{

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        while (opModeIsActive()){
            //detection
            //move forward
           moveAuto(0, 10, 10, 5);
           idle();
        }
    }
}
