package org.firstinspires.ftc.team13580.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team13580.RobotHardware;

@Autonomous(name="Robot red: Auto Park Short", group="Robot")
public class AutoParkShortAuto extends LinearOpMode {
    RobotHardware robot= new RobotHardware(this);
    private ElapsedTime runtime= new ElapsedTime();

    @Override
    public void runOpMode(){
        robot.init();
        waitForStart();

        //strafe tp the right  2 squares
        robot.setDrivePower(0.6,0.6,0.6,0.6);
        runtime.reset();
        while (opModeIsActive()&&(runtime.seconds()<0.15)){
            telemetry.addData("Path", "Leg1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);


        robot.setDrivePower(0,0,0,0);

        telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
        telemetry.update();
        sleep(1000);



    }
}
