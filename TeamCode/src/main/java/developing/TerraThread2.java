package developing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import telefunctions.Stage;
import util.CodeSeg;


public class TerraThread2 implements Runnable{

    CodeSeg cs;
    Stage st;
    public boolean executing = true;
    public int refreshRate = 100; // hertz


    public TerraThread2(CodeSeg run, Stage stop){
        this.cs = run;
        this.st = stop;
    }

    public TerraThread2(CodeSeg run){
        this.cs = run;
        this.st = new Stage() {
            @Override
            public boolean run(double in) {
                return false;
            }
        };
    }




    public void changeRefreshRate(int rf){
        refreshRate = rf;
    }

    public synchronized void stop() {
        this.executing = false;
    }

    private synchronized boolean isExecuting() {
        return this.executing;
    }

    public void update() {
        cs.run();
        if(st.run(0)){
            stop();
        }
    }

    @Override
    public void run() {
        while (isExecuting()) {
            update();
            try {Thread.sleep(1000/refreshRate); } catch (InterruptedException e) {}
        }
    }


}
