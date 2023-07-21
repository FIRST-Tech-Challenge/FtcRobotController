package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.BlueLeftHigh;
import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;
//@Disabled

@Config
@Autonomous(name = "BlueLeftAutoCycleboost")


public class BlueLeftAutoCycleBoost extends LinearOpMode {
    public void runOpMode() {
        //construct composition classes
        PwPRobot robot = new PwPRobot(this,false);
        BlueLeftHigh autoRunner = new BlueLeftHigh(true,this, robot);
        //initialize robot
        autoRunner.init();
        //manually reset time
        time=0;
        //set up for breaking out of while loop for abort
        abort:
        //27.2 for time abort
        while ((time < 27.2 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!isStopRequested()) {
            //preload sequence
            autoRunner.preload();
            //cycle 5 times
            for (int i = 0; i < 5; i++) {
                if (!autoRunner.pick(i)) {
                    //break out of the program
                    break abort;
                }
                if(i==3){
                    //artificially double loop frequency of rr
                    robot.roadrun.update();
                }
                autoRunner.drop(i);
            }
            //update states of robot
            autoRunner.update();
        }
        //reset the queuer for new queue sequence
        robot.done();
        robot.queuer.reset();
        robot.done();
        //parking sequence, 29.8 for time stop program
        while ((time < 29.8 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!isStopRequested()) {
            //park
            autoRunner.park();
            //update states of robot
            autoRunner.update();
        }
        //stop program
        robot.stop();
        stop();
    }
}
