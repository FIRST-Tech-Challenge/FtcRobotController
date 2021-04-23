package autofunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import globalfunctions.TerraThread;
import util.Stage;
import util.CodeSeg;

public class RobotFunctionsHandler {
    public int rfsIndex = 0;
    public int rfsQueueIndex = 0;
    public ArrayList<CodeSeg> rfs = new ArrayList<>();
    public ArrayList<Boolean> isRf = new ArrayList<>();
    public ArrayList<CodeSeg> rfsQueue = new ArrayList<>();

    public LinearOpMode op;

    public CodeSeg updateCode = new CodeSeg() {
        @Override
        public void run() {
            if(rfsIndex < rfs.size()) {
                if(rfsQueue.size() > rfsQueueIndex){
                    rfsQueue.get(rfsQueueIndex).run();
                    rfsQueueIndex++;
                }
            }
        }
    };

    public Stage exit = new Stage() {
        @Override
        public boolean run(double in) {
            return op.isStopRequested();
        }
    };


    public TerraThread rfsThread;



    public void start(LinearOpMode op){
        this.op = op;
        rfsThread = new TerraThread(updateCode, exit);
//        rfsThread = new TerraThread(updateCode);
        rfsThread.changeRefreshRate(10);
        Thread t = new Thread(rfsThread);
        t.start();
    }

    public void stop(){
        rfsThread.stop();
    }

    public void update(){
        if(rfsIndex < rfs.size() - 1 && isRf.get(rfsIndex+1)) {
            rfsQueue.add(rfs.get(rfsIndex+1));
            rfsIndex++;
        }
        rfsIndex++;
    }


    public void addRFs(CodeSeg... segs){
        rfs.add(combineSegs(segs));
        isRf.add(true);
    }
    public void notRF(){
        rfs.add(nullCode());
        isRf.add(false);
    }

    public static CodeSeg nullCode(){ return new CodeSeg() {
        @Override
        public void run() {

        }
    }; }
    public static CodeSeg combineSegs(final CodeSeg[] segs){
        return new CodeSeg() {
            @Override
            public void run() {
                for(CodeSeg seg:segs) {
                    seg.run();
                }
            }
        };
    }
}
