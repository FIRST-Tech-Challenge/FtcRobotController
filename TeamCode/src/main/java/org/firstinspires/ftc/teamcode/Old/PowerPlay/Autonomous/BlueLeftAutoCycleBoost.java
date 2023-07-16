package org.firstinspires.ftc.teamcode.Old.PowerPlay.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;
//@Disabled

@Config
@Autonomous(name = "BlueLeftAutoCycleboost")


public class BlueLeftAutoCycleBoost extends LinearOpMode {
    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this,false);
//        sleep(500);
        BlueLeftHigh autoRunner = new BlueLeftHigh(true,this, robot);
        autoRunner.init();
        time=0;
        abort:
        while ((time < 27.2 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!isStopRequested()) {
            autoRunner.preload();
            for (int i = 0; i < 5; i++) {
                if (!autoRunner.pick(i)) {
                    break abort;
                }
                if(i==3){
                    robot.roadrun.update();
                }
                autoRunner.drop(i);
            }
            autoRunner.update();
        }
        robot.done();
        robot.queuer.reset();
        robot.done();
        while ((time < 29.8 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!isStopRequested()) {
            autoRunner.park();
            autoRunner.update();
        }
        robot.stop();
        stop();
    }
}
