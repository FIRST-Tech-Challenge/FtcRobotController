package util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ThreadHandler {
    TerraThread teleThread;
    TerraThread autoThread;

    public void startTeleThread(CodeSeg cs, int updateMs){
        teleThread = new TerraThread(cs);
        teleThread.changeMs(updateMs);
        Thread t = new Thread(teleThread);
        t.start();
    }
    public void startAutoThread(CodeSeg cs, LinearOpMode op, int updateMs){
        autoThread = new TerraThread(cs);
        autoThread.changeToAuton(op);
        autoThread.changeMs(updateMs);
        Thread t = new Thread(autoThread);
        t.start();
    }
    public void stopTeleThread(){
        if(teleThread != null) {
            teleThread.stop();
        }
    }
    public void stopAutoThread(){
        if(autoThread != null) {
            autoThread.stop();
        }
    }
}
