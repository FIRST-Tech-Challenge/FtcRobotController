package util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ThreadHandler {
    TerraThread teleThread;
    TerraThread autoThread;

    public void startTeleThread(CodeSeg cs){
        teleThread = new TerraThread(cs);
        Thread t = new Thread(teleThread);
        t.start();
    }
    public void startAutoThread(CodeSeg cs, LinearOpMode op){
        autoThread = new TerraThread(cs);
        autoThread.changeToAuton(op);
        Thread t = new Thread(autoThread);
        t.start();
    }
    public void stopTeleThread(){
        if(teleThread!= null) {
            teleThread.stop();
        }
    }
    public void stopAutoThread(){
        if(autoThread != null) {
            autoThread.stop();
        }
    }
}
