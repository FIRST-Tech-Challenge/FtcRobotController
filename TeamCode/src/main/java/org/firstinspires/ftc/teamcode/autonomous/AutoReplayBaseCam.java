package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Point;

import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.bots.UltimateBot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

public class AutoReplayBaseCam extends AutoBase {

    @Override
    protected void preStart() {
        super.preStart();
        String routeName = getModeName();
        if (!routeName.isEmpty()) {
            loadRoute(routeName);
            if (this.selectedRoute != null){
                this.setOpModeSide(this.selectedRoute.getName());
            }
        }
    }

    @Override
    protected void act() {
        super.act();
        if (opModeIsActive()) {
            runRoute();
        }
    }

    @Override
    protected void initBot() {
        this.bot = new FrenzyBot(this.getOpModeSide());
    }

    @Override
    protected void initLocator() {
        this.locator = VSlamOdometry.getInstance(hardwareMap);
        this.locator.init(new Point(startX, startY), initHead);
        startLocator(locator);
    }
}
