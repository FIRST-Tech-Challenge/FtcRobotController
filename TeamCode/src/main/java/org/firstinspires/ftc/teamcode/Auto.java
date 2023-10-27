package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous")
public class Auto extends LinearOpMode {
    private ElapsedTime runTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = new Hardware(this);
        hw.init(hardwareMap);

        telemetry.addData("Auto: ", "ready for start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            //do stuff
            if(runTime.time() < 5.0){
                hw.drive(0.5, 20);
            }

            if(runTime.time() == 30.0){
                break;
            }
            //cycle every 25 milliseconds, to prevent memory death --> 40 cycles/s
            sleep(25);
        }

        hw.drive(0,0);
    }
}
