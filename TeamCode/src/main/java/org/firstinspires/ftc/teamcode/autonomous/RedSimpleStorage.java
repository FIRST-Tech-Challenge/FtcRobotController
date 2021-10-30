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


@Autonomous(name="Red Simple Storage", group ="15173")
public class RedSimpleStorage extends AutoBase {
    private static String TAG = "RedSimpleStorage";

    @Override
    protected void initBot() {
        setOpModeSide(AutoRoute.NAME_RED);
        this.bot = new FrenzyBot(this.getOpModeSide());
    }

    @Override
    protected void initLocator() {
        startX = 40;
        startY = 10;
        initHead = 180;
        this.locator = VSlamOdometry.getInstance(hardwareMap);
        this.locator.init(new Point(startX, startY), initHead);
        startLocator(locator);
    }

    @Override
    protected void act() {
        //even though we are not using detector in this mode, since it is started
        // in the constructor, we want to close it.
        GameElement detected = this.bot.getDetection();

        Log.d(TAG, String.format("Detected %s", detected.toString()));

        //move sideways to the storage area
        //define a point in the storage area
        Point target = new Point(1, 45);
        //build profile for the robot to move to the storage area using diagonal strategy
        BotMoveProfile profile = BotMoveProfile.bestRoute(this.bot, (int)locator.getCurrentX(), (int)locator.getCurrentY(), target,
                RobotDirection.Optimal, 0.7, MoveStrategy.Diag, -1, locator);
        //move the robot diagonally using the profile object.
        diag(profile);
        //let it sit, as everything is async
        waitToStartStep(3000);

    }
}
