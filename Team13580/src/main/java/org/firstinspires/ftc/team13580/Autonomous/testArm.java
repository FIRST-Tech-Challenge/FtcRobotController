package org.firstinspires.ftc.team13580.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team13580.RobotHardware;

@Autonomous(name="testArm", group="Robot")
public class testArm extends LinearOpMode {
    RobotHardware robot= new RobotHardware(this);
    private ElapsedTime runtime= new ElapsedTime();

    @Override
    public void runOpMode(){
        robot.init();
        waitForStart();
        robot.setHandPositions(1);
        while(opModeIsActive()&& (runtime.seconds()<0.5)){
            telemetry.addData("Path", "Leg1: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);
        //robot.encoderArm(17,10);
        //robot.encoderDrive(0.2,40,40,40,40,17);
    }
}

