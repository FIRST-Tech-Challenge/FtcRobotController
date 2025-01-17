package org.firstinspires.ftc.teamcode.opmode.autos;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;
import org.firstinspires.ftc.teamcode.utils.BT.BTRecordingController;

import java.io.File;
import java.io.IOException;

public class recorder extends periodOpMode {
    BTRecordingController controller;
    @Override
    public void initialize() {
        final String file_name="11226_rec";
        final int maxIterations=20*30;//Hz * Sec
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
