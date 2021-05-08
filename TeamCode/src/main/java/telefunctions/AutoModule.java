package telefunctions;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import autofunctions.Path;
import developing.RobotFunctions2;
import global.TerraBot;
import globalfunctions.TerraThread;
import globalfunctions.Constants;
import util.CodeSeg;
import util.Stage;

public class AutoModule {

    //List of stages in the automodule
    public ArrayList<Stage> stages = new ArrayList<>();
    //Current stage that the automodule is on
    public int stageNum = 0;
    //Timer for internal methods
    public ElapsedTime timer = new ElapsedTime();

    //Is the automodule pausing?
    public boolean pausing =   false;

    public RobotFunctions2 rfs = new RobotFunctions2();

    //Code that updates in the loop
    public CodeSeg updateCode = () -> {
        Stage s = stages.get(stageNum);
        if (s.run(timer.seconds())) {
            stageNum+=1;
            timer.reset();
        }
        if (stageNum == (stages.size())) {
            stageNum = 0;
        }
    };

    //Thread for automodules to run in
    public TerraThread autoModuleThread;
    //Has the automodule been initalized yet?
    public boolean inited = false;
    //Initializes the automodule with a terrabot
    public void init(TerraBot bot){
        this.rfs.init(bot);
    }
    //Starts the automodule and unpauses it
    public void start(){
        //If the thread hasnt been inited yet
        if(!inited) {
            //Define the automodule thread
            autoModuleThread = new TerraThread(updateCode);
            autoModuleThread.changeRefreshRate(Constants.AUTOMODULE_REFRESH_RATE);
            inited = true;
            Thread t = new Thread(autoModuleThread);
            t.start();
        }
        //Set pausing to false
        pausing = false;
    }
    //Is the automodule executing?
    public boolean isExecuting(){
        return autoModuleThread.executing;
    }
    //Stop the automodule
    public void stop(){
        autoModuleThread.stop();
    }

    //Add stage to control the wobble goal extender
    public void addWGE(double pos){
        stages.addAll(rfs.moveWGE(pos));
    }

    //Add stage to move the wobble goal arm to a certian pos
    public void addWGA(double deg, double pow){
        stages.addAll(rfs.moveWGA(deg, pow));
    }
    //Add stage to hold the wobble goal and pause the automodule
    public void holdWobbleGoalAndPause(){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                pausing = true;
                return true;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                rfs.bot.moveArm(rfs.bot.getRestPowArm());
                return !pausing;
            }
        });
    }
    //Add Stage to close/open the claw
    public void addClaw(int idx){
        stages.addAll(rfs.moveClaw(idx));
    }
    //Add a pause
    public void addPause() {
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                pausing = true;
                return true;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                return !pausing;
            }
        });
    }
    //Waits for the amount of time specified
    public void addWait(final double t) {
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                return in > t;
            }
        });
    }
    //Adds a custom code block
    public void addCustom(final CodeSeg cs){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                cs.run();
                return true;
            }
        });
    }
    //Turs to the goal to shoot
    public void addTurnToGoal(){
        stages.addAll(rfs.turnToGoal());
    }

    //Changes autoaimer mode to certain mode
    public void changeAutoAimerMode(int mode){
        stages.addAll(rfs.changeAAMode(mode));
    }
}
