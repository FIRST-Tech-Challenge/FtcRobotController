package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AbstractOpMode extends LinearOpMode {
    private AbstractRobot robot;
    public abstract AbstractRobot instantiateRobot();

    public AbstractRobot getRobot() {
        return robot;
    }

    public void onInit() {
    }

    public void onStop() {
        super.stop();
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
