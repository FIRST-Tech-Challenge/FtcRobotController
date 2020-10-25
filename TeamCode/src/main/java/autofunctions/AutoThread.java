package autofunctions;

import util.CodeSeg;


public class AutoThread implements Runnable{
    private boolean executing = false;
    CodeSeg cs;

    public void init(CodeSeg cs){
        this.cs = cs;
    }
    public synchronized void stop() {
        this.executing = true;
    }

    private synchronized boolean isExecuting() {
        return !this.executing;
    }
    @Override
    public void run() {
        while (isExecuting()){
            cs.run();
        }
    }
}
