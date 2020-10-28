package util;

import java.util.ArrayList;

import telefunctions.TeleThread;
import util.CodeSeg;

public class ThreadHandler {
    TeleThread teleThread;

    public void startTeleThread(CodeSeg cs){
        teleThread = new TeleThread();
        teleThread.init(cs);
        Thread t = new Thread(teleThread);
        t.start();
    }
    public void stopTeleThread(){
        teleThread.stop();
    }
}
