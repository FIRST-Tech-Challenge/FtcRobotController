package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Point;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CVRec.GameElement;
import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;


@Autonomous(name="Red Detect Storage", group ="15173")
@Disabled
public class RedDetect extends AutoBase {
    private static String TAG = "RedSimpleStorage";

    @Override
    protected void initBot() {
        setOpModeSide(AutoRoute.NAME_RED);
        this.bot = new FrenzyBot(this.getOpModeSide());
    }

    @Override
    protected void initLocator() {
        //this is where we start
        startX = 30;
        startY = 10;
        initHead = 180;
        this.locator = VSlamOdometry.getInstance(hardwareMap, VSlamOdometry.THREAD_INTERVAL, startX, startY, (int)initHead);
        this.locator.setCoordinateAdjustmentMode(getOpModeSide());
        startLocator(locator);
    }

    @Override
    protected void act() {
        //even though we are not using detector in this mode, since it is started
        // in the constructor, we want to close it.
        GameElement detected = this.bot.getDetection();

        Log.d(TAG, String.format("Detected %s", detected.toString()));

        Point moveFrwd = new Point(30, 15);

        BotMoveProfile profileMove = BotMoveProfile.bestRoute(this.bot, (int)locator.getCurrentX(), (int)locator.getCurrentY(), moveFrwd,
                RobotDirection.Optimal, 0.5, MoveStrategy.Straight, -1, locator);

        moveStraight(profileMove);
        waitToStartStep(2000);

        Point target = new Point(38, 30);

        BotMoveProfile profileSpin = BotMoveProfile.bestRoute(this.bot, (int)locator.getCurrentX(), (int)locator.getCurrentY(), target,
                RobotDirection.Optimal, 0.5, MoveStrategy.SpinNStraight, 135, locator);

        if (detected == GameElement.BarcodeLevel2){
            ((FrenzyBot)bot).liftToMid();
        }
        else if (detected == GameElement.BarcodeLevel3){
            ((FrenzyBot)bot).liftToUpper();
        }

        //advance to the tower
        spin(profileSpin);

        waitToStartStep(2000);

        BotMoveProfile profileStraight = BotMoveProfile.bestRoute(this.bot, (int)locator.getCurrentX(), (int)locator.getCurrentY(), target,
                RobotDirection.Optimal, 0.6, MoveStrategy.Straight, -1, locator);

        moveStraight(profileStraight);

        waitToStartStep(2000);

        //drop
        ((FrenzyBot)bot).dropElement();

        waitToStartStep(1000);

        ((FrenzyBot)bot).resetDropper();
        ((FrenzyBot)bot).liftToLower();

        BotMoveProfile profileSpinToWarehouse = BotMoveProfile.bestRoute(this.bot, (int)locator.getCurrentX(), (int)locator.getCurrentY(), target,
                RobotDirection.Optimal, 0.7, MoveStrategy.SpinNStraight, 90, locator);


        spin(profileSpin);

        while (opModeIsActive()){
            telemetry.addData("X", "%.3f", locator.getCurrentX());
            telemetry.addData("Y", "%.3f", locator.getCurrentY());
            telemetry.addData("Heading", "%.3f", locator.getOrientation());
            telemetry.update();
        }

//        Point warehouse = new Point(120, 30);
//
//        BotMoveProfile profileStraightToWarehouse = BotMoveProfile.bestRoute(this.bot, (int)locator.getCurrentX(), (int)locator.getCurrentY(), warehouse,
//                RobotDirection.Optimal, 0.8, MoveStrategy.Straight, -1, locator);
//
//        moveStraight(profileStraightToWarehouse);
//
//        waitToStartStep(5000);

    }
}
