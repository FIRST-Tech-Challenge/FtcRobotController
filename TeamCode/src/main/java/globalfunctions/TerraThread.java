package globalfunctions;

import util.Stage;
import util.CodeSeg;


//Used to make threads
public class TerraThread implements Runnable {

    CodeSeg cs;
    Stage st;
    public boolean executing = true;
    public int refreshRate = 100; // hertz

    //Define thread with codeseg to run and stage to stop
    public TerraThread(CodeSeg run, Stage stop){
        this.cs = run;
        this.st = stop;
    }
    //Define thread with just codeseg to run
    public TerraThread(CodeSeg run){
        this.cs = run;
        this.st = new Stage() {
            @Override
            public boolean run(double in) {
                return false;
            }
        };
    }



    //Change the refresh rate of the thread
    public void changeRefreshRate(int rf){
        refreshRate = rf;
    }
    //Stop the thread
    public synchronized void stop() {
        this.executing = false;
    }
    //Is the thread executing
    private synchronized boolean isExecuting() {
        return this.executing;
    }
    //update the thread
    public void update() {
        cs.run();
        if(st.run(0)){
            stop();
        }
    }

    //Overrides the run method of runnable and sleeps for the desired refresh rate
    @Override
    public void run() {
        while (isExecuting()) {
            update();
            Sleep.trySleep(() -> Thread.sleep(1000/refreshRate));
//            try {Thread.sleep(1000/refreshRate); } catch (InterruptedException e) {}
        }
    }


}
