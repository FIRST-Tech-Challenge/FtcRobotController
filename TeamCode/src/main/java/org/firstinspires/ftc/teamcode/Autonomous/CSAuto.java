package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class CSAuto extends LinearOpMode {
    protected TrajectorySequence toSpike, spikeToBackdrop,backdropToStack, stackToBackDrop;
    protected boolean restack = true, ultraSafe = true, ultraSteal = true;
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
