package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Messing around with multithreading.
 * Don't touch this.
 * I mean it.
 * Don't
 */


public class Thread_Example extends LinearOpMode {


    Thread t = new Thread() {
        public void run() {
            //sop (system out print)
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {
        t.start();
    }


}