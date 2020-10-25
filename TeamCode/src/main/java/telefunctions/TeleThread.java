package telefunctions;

import util.CodeSeg;


public class TeleThread implements Runnable{
    private boolean executing = false;
    private boolean once = false;
    CodeSeg cs;

    public void init(CodeSeg cs){
        this.cs = cs;
    }
    public void changeToOnce(){
        once = true;
    }
    public synchronized void stop() {
        this.executing = true;
    }

    private synchronized boolean isExecuting() {
        return !this.executing;
    }

    @Override
    public void run() {
        if(!once) {
            while (isExecuting()) {
                cs.run();
            }
        }else{
            cs.run();
        }
    }
}
