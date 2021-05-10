package developing;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import autofunctions.RobotFunctionsHandler;
import global.TerraBot;
import globalfunctions.Constants;
import globalfunctions.TerraThread;
import util.CodeSeg;
import util.Stage;

public class AutoModule2 {

    //Current stage that the automodule is on
    public int stageNum = 0;
    //Is the automodule pausing?
    public boolean pausing = false;
    public RobotFunctionsHandler2 rfh;
    //List of indexes to pause at
    public ArrayList<Integer> pauses = new ArrayList<>();
    //Stage numbers when defining automodule
    public int defineStageNum = 0;

    //Initialize the automodule with the robot function handler
    public void init(RobotFunctionsHandler2 rfh){
        this.rfh = rfh;
        addPause();
    }


    //Starts the automodule and unpauses it
    public void start(){
        //Set pausing to false
        pausing = false;
    }
    //Updates the automodule
    public void update(){
        //if the automodule is running and it should not pause then add the current robot function to the queue
        if(!pausing && !shouldPause()) {
            rfh.update(stageNum);
            next();
        }else if(!pausing){ //If the automodule is running and it should pause then set pausing to true and go to the next stage
            pausing = true;
            next();
        }
    }

    //Goes to next robot function
    public void next(){
        if(stageNum < rfh.size()) {
            stageNum++;
        }else {
            stageNum = 0;
        }
    }

    //Is the automodule executing?
    public boolean isExecuting(){
        return !pausing;
    }

    //Should the automodule pause now?
    public boolean shouldPause(){
        for(int i:pauses){
            if(stageNum == i){
                return true;
            }
        }
        return false;
    }

    //Add a new robot function
    @SafeVarargs
    public final void add(ArrayList<Stage>... stages){
        rfh.addRFs(defineStageNum, stages);
        defineStageNum++;
    }
    //Add a pause
    public void addPause(){
        pauses.add(defineStageNum);
        defineStageNum++;
    }

}
