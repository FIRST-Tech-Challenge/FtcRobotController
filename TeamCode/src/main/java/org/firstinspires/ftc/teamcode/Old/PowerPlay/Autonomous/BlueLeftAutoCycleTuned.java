package org.firstinspires.ftc.teamcode.Old.PowerPlay.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.BlueLeftHigh;
import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;
//@Disabled

@Config
@Autonomous(name = "BlueLeftAutoCycleTuned")


public class BlueLeftAutoCycleTuned extends LinearOpMode {


    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this,false);
        BlueLeftHigh autoRunner = new BlueLeftHigh(false,this, robot);
        sleep(500);
        autoRunner.init();
        abort:
        while ((time < 27.5 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!isStopRequested()) {
            autoRunner.preload();
            for (int i = 0; i < 5; i++) {
                if (!autoRunner.pick(i)) {
                    break abort;
                }
                if(i==3){
//                    robot.roadrun.update();
                }
                autoRunner.drop(i);
            }
            autoRunner.update();
        }
        robot.done();
        robot.queuer.reset();
        robot.done();
        while ((time < 29.9 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!isStopRequested()) {
            autoRunner.park();
            autoRunner.update();
        }
        stop();
    }
}
