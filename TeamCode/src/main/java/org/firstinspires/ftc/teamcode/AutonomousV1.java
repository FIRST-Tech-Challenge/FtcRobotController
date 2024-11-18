package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.OdometryBot;
import org.firstinspires.ftc.teamcode.bots.PinchBot;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "AutoTestBlue", group = "Auto")
public class AutonomousV1 extends LinearOpMode {
    private PinchBot odometryBot = new PinchBot(this);
    @Override
    public void runOpMode() throws InterruptedException {

        odometryBot.isAuto = true;
        odometryBot.init(hardwareMap);


        waitForStart();


            ElapsedTime timer = new ElapsedTime();
            int milliseconds = 1350;

            while(timer.milliseconds()<1350){
                odometryBot.driveByHandFieldCentric(0.5, 0,
                        0, false, 0, 0, 0, false);
            }
            timer.reset();
//        while(timer.milliseconds()<1950){
//            odometryBot.driveByHandFieldCentric(0, -0.5,
//                    0, false, 0, 0, 0, false);
//        }
//            timer.reset();
//        while(timer.milliseconds()<700){
//            odometryBot.driveByHandFieldCentric(0.5, 0,
//                    0, false, 0, 0, 0, false);
//        }
//        timer.reset();
//        while(timer.milliseconds()<1950){
//            odometryBot.driveByHandFieldCentric(0, 0.5,
//                    0, false, 0, 0, 0, false);
//        }

        getBlock();
        getBlock();



        odometryBot.driveByHandFieldCentric(0, 0,
                0, false, 0, 0, 0, false);


        telemetry.update();


    }
    public void getBlock(){
        ElapsedTime timer = new ElapsedTime();
        while(timer.milliseconds()<1950){
            odometryBot.driveByHandFieldCentric(0, -0.5,
                    0, false, 0, 0, 0, false);
        }
        timer.reset();
        while(timer.milliseconds()<800){
            odometryBot.driveByHandFieldCentric(0.5, 0,
                    0, false, 0, 0, 0, false);
        }
        timer.reset();
        while(timer.milliseconds()<1950){
            odometryBot.driveByHandFieldCentric(0, 0.5,
                    0, false, 0, 0, 0, false);
        }
    }
}