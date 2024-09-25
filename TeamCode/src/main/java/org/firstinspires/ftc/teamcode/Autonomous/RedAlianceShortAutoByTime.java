package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Robot red: Auto Drive By Time", group="Robot")
@Disabled
public class RedAlianceShortAutoByTime extends LinearOpMode {
    RobotHardware robot= new RobotHardware(this);
    private ElapsedTime runtime= new ElapsedTime();
    @Override
    public void runOpMode(){
        robot.init();
        waitForStart();

        robot.setDrivePower(0.5,0.5,-0.5,-0.5);
        runtime.reset();
        while (opModeIsActive()&&(runtime.seconds()<1.3)){
            telemetry.addData("Path", "Leg1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.setDrivePower(0.6,0.6,0.6,0.6);
        runtime.reset();
        while(opModeIsActive()&&(runtime.seconds()<3.0)){
            telemetry.addData("Path","Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.setDrivePower(-0.5,-0.5,0.5,0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds()<1.3)){
            telemetry.addData("Path" , "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.setDrivePower(0,0,0,0);

        telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
        telemetry.update();
        sleep(1000);



    }
}
