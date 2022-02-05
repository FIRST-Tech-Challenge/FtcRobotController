package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TestTwo extends LinearOpMode {
    HWC robot = new HWC(hardwareMap, telemetry);
    boolean done = false;
    boolean drive2ASH = true;
    boolean drive2WH = false;
    boolean deliver = false;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(!done){
            if(drive2ASH){
                // Drive and arm up SAME TIME

            } else if(deliver){
                // Servos out
            } else if(drive2WH){
                // Drive and arm down SAME TIME
            }

            if(drive2WH = false){
                done = true;
            }
        }


    }
}
