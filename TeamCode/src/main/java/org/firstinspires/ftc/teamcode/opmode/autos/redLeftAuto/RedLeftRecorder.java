package org.firstinspires.ftc.teamcode.opmode.autos.redLeftAuto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.autos.Recorder;

@TeleOp(name = "RedLeftRecorder",group = "recorders")
public class RedLeftRecorder extends Recorder {
    public final static int maxIterations=20*30;
    public final static String file_name="11226_RedLeft_Auto";
    @Override
    protected int maxIterations() {
        return maxIterations;
    }

    @Override
    protected String file_name() {
        return file_name;
    }
}
