package telefunctions;

import util.CodeSeg;

public class ThreadHandler {
    public void startTeleThread(CodeSeg cs){
        TeleThread teleThread = new TeleThread();
        teleThread.init(cs);
        teleThread.run();
    }
}
