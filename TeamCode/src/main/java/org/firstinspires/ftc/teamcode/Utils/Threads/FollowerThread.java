package org.firstinspires.ftc.teamcode.Utils.Threads;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class FollowerThread extends Thread {

    Follower f;
    LinearOpMode opMode;
    public FollowerThread(LinearOpMode opMode,Follower follower) {
        f = follower;
        this.opMode = opMode;
    }

    @Override
    public void run() {
        super.run();
        while(opMode.opModeIsActive() || opMode.opModeInInit()) {
            f.update();
            /*long start = System.currentTimeMillis();
            while(System.currentTimeMillis()-start<5 && (opMode.opModeIsActive() || opMode.opModeInInit()));*/
        }
    }
}
