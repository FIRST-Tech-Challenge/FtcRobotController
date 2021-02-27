package org.firstinspires.ftc.teamcode.toolkit.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class UpliftAuto extends LinearOpMode {

    public abstract void initHardware();

    public abstract void initAction();

    public abstract void body() throws InterruptedException;

    public abstract void exit() throws InterruptedException;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initializing", "Started");
        telemetry.update();

        initHardware();
        initAction();

        telemetry.addData("Initializing", "Finished");
        telemetry.update();

        waitForStart();

        telemetry.addData("Body", "Started");
        telemetry.update();

        body();

        telemetry.addData("Body", "Finished");
        telemetry.update();

        exit();
    }
}