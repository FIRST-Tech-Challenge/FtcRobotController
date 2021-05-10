package telefunctions;

import java.util.ArrayList;

import autofunctions.RobotFunctionsHandler;
import util.Stage;

public class AutoModule {

    //Current stage that the automodule is on
    public int stageNum = 0;
    //Is the automodule pausing?
    public boolean pausing = false;
    //Robot Functions Handler to use
    public RobotFunctionsHandler rfh;
    //List of indexes to pause at
    public ArrayList<Integer> pauses = new ArrayList<>();
    //Stage numbers when defining automodule
    public int defineStageNum = 0;
    //Stage number to start at
    public int startStageNum = 0;

    //Initialize the automodule with the robot function handler and start by pausing
    public void init(RobotFunctionsHandler rfh){
        this.rfh = rfh;
        //Start the stage num index from the size of the current handler
        startStageNum = rfh.size();
        defineStageNum = startStageNum;
        stageNum = startStageNum;
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
        if(stageNum < (defineStageNum)) {
            stageNum++;
        }else {
            stageNum = startStageNum;
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

    //Add new robot function(s)
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
