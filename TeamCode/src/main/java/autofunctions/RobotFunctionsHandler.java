package autofunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;

import globalfunctions.Constants;
import globalfunctions.TerraThread;
import util.CodeSeg;
import util.Stage;

public class RobotFunctionsHandler {
    /* This is a description of how robot functions work
    * First a new thread is created that always runs in the background
    * This thread is in charge of running whatever robot functions are in the queue
    * Whenever it detects that there are items in the queue it runs all of them until
    * they are all done and then it clears the queue
    * To place items in the queue an ordered list of robot functions is created
    * In this list each item is associated with an index
    * When you define an automodule the automodule fills up the list with robot functions in order
    * one after the other unless there is a pause where it would skip that index
    * So for automodules (a1,a2,...) it would look something like this
    * index | value
    * 0 | a1.rf1   - first robot function
    * 1 | a1.rf2
    * 2 | null     - pause
    * 3 | a1.rf3
    * 4 | null     - 2nd pause
    * 5 | a1.rf4   - final robot function
    * 6 | a2.rf1   - first robot function of a2
    * 7 | a2.rf2
    * and so on
    * Each automodule would then tell this robot function handler to add its own robot functions to
    * the queue when the automodule starts
    * In autonomous the same thing happens except for anything that is not an robot function there is nothing (Path p)
    * 0 | p.rf1   - first robot function
    * 1 | null    -moving or stopping
    * 2 | null
    * 3 | null
    * 4 | p.rf2   - 2nd robot function
    * and so on ...
    * */

    //Robot Function queue index
    public int rfsQueueIndex = 0;
    //Map of all robot functions defined
    public volatile HashMap<Integer, ArrayList<Stage>> allRFs = new HashMap<>();
    //List of all robot functions currently in the queue
    public volatile ArrayList<Stage> rfsQueue = new ArrayList<>();
    //Timer
    public ElapsedTime timer = new ElapsedTime();
    //Update code for running the queue
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
    //Thread for queue
    public TerraThread rfsThread;


    //Start the queue thread
    public void start(){
        rfsThread = new TerraThread(updateCode);
        rfsThread.changeRefreshRate(Constants.ROBOT_FUNCTIONS_REFRESH_RATE);
        Thread t = new Thread(rfsThread);
        t.start();
    }
    //Stop the thread
    public void stop(){
        rfsThread.stop();
    }
    //Update by adding the robot function at the specified index to the queue
    public void update(int curIndex){
        ArrayList<Stage> curRf = allRFs.get(curIndex);
        if(curRf != null){
            rfsQueue.addAll(curRf);
        }
    }

    //Add robot functions based on index
    @SafeVarargs
    public final void addRFs(int index, ArrayList<Stage>... stages){
        allRFs.put(index, combineStages(stages));
    }

    //Combine stages
    @SafeVarargs
    public static ArrayList<Stage> combineStages(ArrayList<Stage>... stages){
        ArrayList<Stage> out = new ArrayList<>();
        for(ArrayList<Stage> stages1 :stages) {
            out.addAll(stages1);
        }
        return out;
    }
    //Return the size of the robot functions list
    public int size(){
        return allRFs.size();
    }
}
