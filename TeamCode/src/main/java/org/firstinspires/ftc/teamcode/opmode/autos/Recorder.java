package org.firstinspires.ftc.teamcode.opmode.autos;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.BT.BTRecordingController;

import java.io.File;
import java.io.IOException;

public abstract class Recorder extends PeriodicOpMode {
    BTRecordingController controller;
    protected abstract int maxIterations();// default should be 20*30 //Hz * Sec
    protected abstract String file_name();

    @Override
    public void initialize() {
        final String file_name=file_name();
        final int maxIterations=maxIterations();
        try {
            File log = AppUtil.getInstance().getSettingsFile(file_name);
            controller=new BTRecordingController(gamepad1,log,maxIterations);
            m_robot=new RobotContainer(hardwareMap,controller);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    protected void period() {
        controller.addRecord();
    }

    @Override
    protected void endFunction() {
        controller.done();
    }
}
