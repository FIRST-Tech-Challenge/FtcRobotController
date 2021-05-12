package autofunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;

import global.TerraBot;
import globalfunctions.TelemetryHandler;
import util.Line;
import util.Stage;

public class Path {

    //Setpoint and waypoint controllers
    public SetpointController setpointController = new SetpointController();
    public WaypointController waypointController = new WaypointController();
    //Global time since the start
    public ElapsedTime globalTime = new ElapsedTime();
    //The curent index
    public int curIndex = 0;
    //The index used to define
    public int defineIndex = 0;
    //List of positions
    public ArrayList<double[]> poses = new ArrayList<>();
    //List of lines
    public ArrayList<Line> lines = new ArrayList<>();
    //List of posetypes
    public ArrayList<Posetype> posetypes = new ArrayList<>();
    //List of stops
    public ArrayList<Double> stops = new ArrayList<>();
    //The stop index
    public int stopIndex = 0;
    //Telemetry handler
    public TelemetryHandler telemetryHandler = new TelemetryHandler();
    //Is the path executing
    public boolean isExecuting = true;
    //Angle to the goal
    public double angleToGoal = 0;
    //Terrabot
    public TerraBot bot;
    //Constructor with start positions
    public Path(double sx, double sy, double sh){
        poses.add(new double[]{sx, sy, sh});
    }
    public Path(double[] spos){
        poses.add(spos);
    }
    //Add a setpoint in the start and initalize everything
    public void init(TerraBot bot){
        posetypes.add(Posetype.SETPOINT);
        setpointController.init();
        waypointController.init();
        globalTime.reset();
        this.bot = bot;
        addStop(0.01);
    }
    //Add a new position
    public void addNewPose(double x, double y, double h){
        double[] lastPose = poses.get(poses.size() - 1);
        poses.add(new double[]{lastPose[0] + x, lastPose[1] + y, lastPose[2] + h});
        double[] currPose = poses.get(poses.size() - 1);
        lines.add(new Line(lastPose[0], lastPose[1], currPose[0], currPose[1]));
        defineIndex++;
    }
    //Add a waypoint
    public void addWaypoint(double x, double y, double h){
        addNewPose(x,y,h);
        posetypes.add(Posetype.WAYPOINT);
    }
    //Add a setpoint
    public void addSetpoint(double x, double y, double h){
        addNewPose(x,y,h);
        posetypes.add(Posetype.SETPOINT);
    }
    //Add a stop
    public void addStop(double time){
        addNewPose(0,0,0);
        posetypes.add(Posetype.STOP);
        stops.add(time);
    }
    //Add a shoot
    public void addShoot(double x, double y, TerraBot bot){
        addNewPose(x, y, 0);
        posetypes.add(Posetype.SHOOT);
    }
    //Add a robot function
    @SafeVarargs
    public final void addRF(ArrayList<Stage>... stages){
        bot.rfh1.addRFs(defineIndex, stages);
    }
    //Add a robot function to the 2nd thread
    @SafeVarargs
    public final void addRF2(ArrayList<Stage>... stages){
        bot.rfh2.addRFs(defineIndex, stages);
    }
    //Go to the next index
    public void next(){
        resetControls();
        globalTime.reset();
        curIndex++;
        bot.rfh1.update(curIndex);
        bot.rfh2.update(curIndex);
        if(curIndex >= lines.size()){
            end();
        }
    }
    //End the path
    public void end(){
        isExecuting = false;
        curIndex--;
    }
    //Update the controls for waypoint
    public void updateControlsForWaypoint(double[] currentPos, double[] target, Line currentLine){
        waypointController.update(currentPos, target, currentLine);
        if(waypointController.isDone()){
            next();
        }
    }
    //Update the controls for setpoints
    public void updateControlsForSetPoint(double[] currentPos, double[] target, boolean isShoot){
        setpointController.update(currentPos, target);
        if (setpointController.isDone()) {
            if(!isShoot) {
                next();
            }else{
                bot.autoAimer.reached();
            }
        }
    }
    //Reset the controls
    public void resetControls(){
        waypointController.reset();
        setpointController.reset();
    }
    //Update the path
    public double[] update(double[] currentPos){
        double[] target = poses.get(curIndex+1);
        switch (posetypes.get(curIndex+1)){
            case WAYPOINT:
                updateControlsForWaypoint(currentPos, target, lines.get(curIndex));
                return waypointController.getPowers();
            case SETPOINT:
                updateControlsForSetPoint(currentPos, target, false);
                return setpointController.getPowers();
            case STOP:
                if(globalTime.seconds() > stops.get(stopIndex)){
                    next();
                    stopIndex++;
                }
                return new double[]{0,0,0};
            case SHOOT:
                double[] targetSh = poses.get(curIndex+1);
                targetSh = new double[]{targetSh[0], targetSh[1], angleToGoal};
                updateControlsForSetPoint(currentPos, targetSh, true);
                if(!bot.autoAimer.hasPosBeenUpdated()) {
                    angleToGoal = bot.autoAimer.getRobotToGoalAngle(targetSh);
                    bot.autoAimer.setOuttakePos(Arrays.copyOf(targetSh, 2));
                }
                if(bot.autoAimer.isDone){
                    bot.autoAimer.ready();
                    next();
                }
                bot.outtakeWithCalculations(false);
                return setpointController.getPowers();
            default:
                return new double[]{0,0,0};
        }
    }

    //Start the path
    public void start(LinearOpMode op){
        op.telemetry.addData("Resetting", "Odometry");
        op.telemetry.update();
        telemetryHandler.init(op.telemetry, bot);
        bot.odometry.resetAll(poses.get(0));
        bot.angularPosition.resetGyro(poses.get(0)[2]);
        op.telemetry.addData("Starting", "Movement");
        op.telemetry.update();
        globalTime.reset();
        while (op.opModeIsActive() && isExecuting) {
            double[] pows = update(bot.odometry.getAll());
            bot.move(pows[1], pows[0], pows[2]);
            op.telemetry.addData("rfsSize", bot.rfh1.rfsQueue.size());
            op.telemetry.update();
        }
        op.telemetry.addData("Status:", "Done");
        op.telemetry.update();
        bot.move(0,0,0);
        bot.stop();
    }
    //Different position types
    public enum Posetype{
        WAYPOINT,
        SETPOINT,
        STOP,
        SHOOT
    }

}