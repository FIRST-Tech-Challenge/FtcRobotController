package org.firstinspires.ftc.teamcode.opmode.autos.redLeftAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autos.Playback;
@Autonomous(name = "RedLeftPlayback",group = "playback")
public class RedLeftPlayback extends Playback {
    @Override
    protected int maxIterations() {
        return RedLeftRecorder.maxIterations;
    }

    @Override
    protected String file_name() {
        return RedLeftRecorder.file_name;
    }
}
