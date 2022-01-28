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
                bot.initDetectorThread(this.getOpModeSide(), this);
            }
        }
    }

    @Override
    protected void act() {
        super.act();
        if (opModeIsActive()) {
            getOpModeTimer().reset(); //start the timer for the whole opmode
            runRoute(false);
        }
    }

    @Override
    protected void initBot() {
        this.bot = new FrenzyBot(this.getOpModeSide());
    }

    @Override
    protected void initLocator() {
        this.locator = VSlamOdometry.getInstance(hardwareMap, VSlamOdometry.THREAD_INTERVAL, this.selectedRoute.getStartX(), this.selectedRoute.getStartY(), (int) this.selectedRoute.getInitRotation());
        startLocator(locator);
    }

    @Override
    protected void displayStatus(){
        telemetry.addData("Locator", String.format("X: %.2f, Y: %.2f", locator.getCurrentX(), locator.getCurrentY()));
        telemetry.addData("Orientation", locator.getAdjustedCurrentHeading());
        AutoDot result = bot.getCurrentDetectionResult();
        if (result!= null) {
            telemetry.addData("Detection result", result.getDotName());
        }
        telemetry.update();
    }
}
