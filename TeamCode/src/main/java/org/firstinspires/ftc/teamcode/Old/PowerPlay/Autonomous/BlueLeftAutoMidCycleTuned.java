package org.firstinspires.ftc.teamcode.Old.PowerPlay.Autonomous;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;


@Config
@Autonomous(name = "BlueLeftAutoMidCycleTuned")
//temp auto logs
//13.24 E NOBOOST NOROTATED

public class BlueLeftAutoMidCycleTuned extends LinearOpMode {

    public static double dummyP = 3;

    public static double dropX = 28.5, dropY = 21, dropA = toRadians(320), dropET = toRadians(150);

//    public static double pickupX1 = -46, pickupY1 = 10, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
public static double pickupX2 = 63, pickupY2 = 11.5, pickupA2 = toRadians(0), pickupET2 = toRadians(0);

    double[] stackPos = {440, 330, 245, 100, 0};

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this,false);
        BlueLeftMid autoRunner = new BlueLeftMid(false,this, robot);
        sleep(500);
        autoRunner.init();
        abort:
        while ((getRuntime() < 27 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!isStopRequested()) {
            autoRunner.preload();
            for (int i = 0; i < 5; i++) {
                if (!autoRunner.pick(i)) {
                    break abort;
                }
//                if(i==3){
//                    robot.roadrun.update();
//                }
                autoRunner.drop(i);
            }
            autoRunner.update();
        }
        robot.done();
        robot.queuer.reset();
        robot.done();
        while ((getRuntime() < 29.8 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!isStopRequested()) {
            autoRunner.park();
            autoRunner.update();
        }
        stop();
    }
}
