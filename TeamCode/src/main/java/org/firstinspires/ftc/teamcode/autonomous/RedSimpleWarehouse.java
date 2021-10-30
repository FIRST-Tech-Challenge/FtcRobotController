package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Point;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CVRec.GameElement;
import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;


@Autonomous(name="Red Simple Warehouse", group ="15173")
public class RedSimpleWarehouse extends AutoBase {
    private static String TAG = "RedSimpleStorage";

    @Override
    protected void initBot() {
        setOpModeSide(AutoRoute.NAME_RED);
        this.bot = new FrenzyBot(this.getOpModeSide());
    }

    @Override
    protected void initLocator() {
        //position the robot facing the warehouse
        startX = 76;
        startY = 10;
        initHead = 0;
        this.locator = VSlamOdometry.getInstance(hardwareMap);
        this.locator.setCoordinateAdjustmentMode(getOpModeSide());
        this.locator.init(new Point(startX, startY), initHead);
        startLocator(locator);
    }

    @Override
    protected void act() {
        //even though we are not using detector in this mode, since it is started
        // in the constructor, we want to close it.
        GameElement detected = this.bot.getDetection();

        Log.d(TAG, String.format("Detected %s", detected.toString()));

        //define a coordinate in the ware house (same Y as the start, different X)
        Point target = new Point(120, 10);
        //Build profile for the robot to move
        BotMoveProfile profile = BotMoveProfile.bestRoute(this.bot, (int)locator.getCurrentX(), (int)locator.getCurrentY(), target,
                RobotDirection.Optimal, 0.7, MoveStrategy.Straight, -1, locator);
        //move the robot using the profile
        moveStraight(profile);
        //let it sit, as everything is async
        waitToStartStep(3000);
        while (opModeIsActive()){
            telemetry.addData("X", "%.3f", locator.getCurrentX());
            telemetry.addData("Y", "%.3f", locator.getCurrentY());
            telemetry.addData("Heading", "%.3f", locator.getOrientation());
            telemetry.update();
        }

    }
}
