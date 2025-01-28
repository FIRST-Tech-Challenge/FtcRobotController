package org.firstinspires.ftc.teamcode.opmode.autos.blueLeftAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autos.Playback;
import org.firstinspires.ftc.teamcode.opmode.autos.blueLeftAuto.BlueLeftRecorder;

    @Autonomous(name = "BLueLeftPlayback",group = "playback")
    public class BlueLeftPlayback extends Playback {
        @Override
        protected int maxIterations() {
            return BlueLeftRecorder.maxIterations;
        }

        @Override
        protected String file_name() {
            return BlueLeftRecorder.file_name;
        }
    }

