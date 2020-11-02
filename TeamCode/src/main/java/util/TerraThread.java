package util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import util.CodeSeg;


public class TerraThread implements Runnable{
    private boolean executing = false;
    private boolean auton = false;
    CodeSeg cs;
    LinearOpMode op;

    public TerraThread(CodeSeg cs){
        this.cs = cs;
    }

    public void changeToAuton(LinearOpMode o) {
        auton = true;
        op = o;
    }
    public synchronized void stop() {
        this.executing = true;
    }

    private synchronized boolean isExecuting() {
        return !this.executing;
    }

    @Override
    public void run() {
        if(!auton) {
            while (isExecuting()) {
                cs.run();
                try {Thread.sleep(10); } catch (InterruptedException e) {}
            }
        }else{
            while (isExecuting() && op.opModeIsActive()) {
                cs.run();
                try {Thread.sleep(10); } catch (InterruptedException e) {}
            }
        }
    }
}
