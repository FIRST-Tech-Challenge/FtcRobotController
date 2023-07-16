package org.firstinspires.ftc.teamcode.Old.PowerPlay.Autonomous;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "BlueRightAutoMidCycleBoost")


public class BlueRightAutoMidCycleBoost extends LinearOpMode {
    private SampleMecanumDrive roadrun;

    public static double dummyP = 1;

    public static double dropX = -29.5, dropY = 18.5, dropA = toRadians(215), dropET = toRadians(30);

    public static double pickupX1 = -46, pickupY1 = 10, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = -64.69  , pickupY2 = 11, pickupA2 = toRadians(180), pickupET2 = toRadians(180);

    double[] stackPos = {440, 330, 245, 100, 0};

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this,false);
        BlueRightMid autoRunner = new BlueRightMid(true,this, robot);
//        sleep(500);
        autoRunner.init();
        time=0;
        abort:
        while ((getRuntime() < 27 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!isStopRequested()) {
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
        while ((getRuntime() < 29.8 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!isStopRequested()) {
            autoRunner.park();
            autoRunner.update();
        }
        robot.stop();

        stop();
    }
}
