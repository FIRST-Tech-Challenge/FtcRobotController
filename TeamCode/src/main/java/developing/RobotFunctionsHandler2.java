package developing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import global.TerraBot;
import globalfunctions.TerraThread;
import util.CodeSeg;
import util.Stage;

public class RobotFunctionsHandler2 {
    public int rfsQueueIndex = 0;
    public volatile HashMap<Integer, ArrayList<Stage>> allRFs = new HashMap<>();
    public volatile ArrayList<Stage> rfsQueue = new ArrayList<>();

    public ElapsedTime timer = new ElapsedTime();

    public CodeSeg updateCode = () -> {
        if(rfsQueueIndex < rfsQueue.size()){
            Stage s = rfsQueue.get(rfsQueueIndex);
            if (s.run(timer.seconds())) {
                rfsQueueIndex++;
                timer.reset();
            }
        }else{
            rfsQueueIndex = 0;
            rfsQueue.clear();
        }
    };


    public TerraThread rfsThread;



    public void start(){
        rfsThread = new TerraThread(updateCode);
        rfsThread.changeRefreshRate(10);
        Thread t = new Thread(rfsThread);
        t.start();
    }

    public void stop(){
        rfsThread.stop();
    }

    public void update(int curIndex){
        ArrayList<Stage> curRf = allRFs.get(curIndex);
        if(curRf != null){
            rfsQueue.addAll(curRf);
        }
    }


    @SafeVarargs
    public final void addRFs(int index, ArrayList<Stage>... stages){
        allRFs.put(index, combineStages(stages));
    }


    public static ArrayList<Stage> combineStages(ArrayList<Stage>... stages){
        ArrayList<Stage> out = new ArrayList<>();
        for(ArrayList<Stage> stages1 :stages) {
            out.addAll(stages1);
        }
        return out;
    }

    public int size(){
        return allRFs.size();
    }
}
