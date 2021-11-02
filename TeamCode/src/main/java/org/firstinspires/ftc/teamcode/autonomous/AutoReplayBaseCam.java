package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
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
            runRoute(false);
        }
    }

    @Override
    protected void initBot() {
        this.bot = new FrenzyBot(this.getOpModeSide());
        bot.initDetectorThread(this.getOpModeSide(), this);
    }

    @Override
    protected void initLocator() {
        this.locator = VSlamOdometry.getInstance(hardwareMap, VSlamOdometry.THREAD_INTERVAL, this.selectedRoute.getStartX(), this.selectedRoute.getStartY(), (int) this.selectedRoute.getInitRotation());
        startLocator(locator);
    }
}
