package autofunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;

import global.TerraBot;
import globalfunctions.PID;
import globalfunctions.Storage;
import globalfunctions.TelemetryHandler;
import globalfunctions.TimeData;
import util.CodeSeg;
import util.Line;
import util.Vector;

public class Path {


    public SetpointController setpointController = new SetpointController();
    public WaypointController waypointController = new WaypointController();

    public ElapsedTime globalTime = new ElapsedTime();
    public int curIndex = 0;

    public ArrayList<double[]> poses = new ArrayList<>();
    public ArrayList<Line> lines = new ArrayList<>();
    public ArrayList<Posetype> posetypes = new ArrayList<>();
    public RobotFunctionsHandler rfsHandler = new RobotFunctionsHandler();
    public RobotFunctionsHandler wobbleGoalHandler = new RobotFunctionsHandler();

    public ArrayList<Double> stops = new ArrayList<>();
    public int stopIndex = 0;
    public TelemetryHandler telemetryHandler = new TelemetryHandler();

    public boolean isExecuting = true;

    public double angleToGoal = 0;

    public Path(double sx, double sy, double sh){
        poses.add(new double[]{sx, sy, sh});
        init();
    }
    public Path(double[] spos){
        poses.add(spos);
        init();
    }
    public void init(){
        posetypes.add(Posetype.SETPOINT);
        setpointController.init();
        waypointController.init();
        globalTime.reset();
        addStop(0.01);
    }

    public void addNewPose(double x, double y, double h){
        double[] lastPose = poses.get(poses.size() - 1);
        poses.add(new double[]{lastPose[0] + x, lastPose[1] + y, lastPose[2] + h});
        double[] currPose = poses.get(poses.size() - 1);
        lines.add(new Line(lastPose[0], lastPose[1], currPose[0], currPose[1]));

        rfsHandler.notRF();
        wobbleGoalHandler.notRF();
    }
    public void addWaypoint(double x, double y, double h){
        addNewPose(x,y,h);
        posetypes.add(Posetype.WAYPOINT);
    }
    public void addSetpoint(double x, double y, double h){
        addNewPose(x,y,h);
        posetypes.add(Posetype.SETPOINT);
    }

    public void addStop(double time){
        addNewPose(0,0,0);
        posetypes.add(Posetype.STOP);
        stops.add(time);
    }


    public void addShoot(double x, double y, TerraBot bot){
        addNewPose(x, y, 0);
        posetypes.add(Posetype.SHOOT);
    }

    public void addRF(CodeSeg... segs){
        rfsHandler.addRFs(segs);
    }

    public void addWGRF(CodeSeg... segs) {
        wobbleGoalHandler.addRFs(segs);
    }

    public void startRFThread(LinearOpMode op){
       rfsHandler.start(op);
       wobbleGoalHandler.start(op);
    }
    public void stopRFThread(){
       rfsHandler.stop();
       wobbleGoalHandler.stop();
    }

    public void next(){
        resetControls();
        globalTime.reset();
        curIndex++;
        rfsHandler.update();
        wobbleGoalHandler.update();
        if(curIndex >= lines.size()){
            end();
        }
    }

    public void end(){
        isExecuting = false;
        curIndex--;
    }

    public void updateControlsForWaypoint(double[] currentPos, double[] target, Line currentLine){
        waypointController.update(currentPos, target, currentLine);
        if(waypointController.isDone()){
            next();
        }
    }

    public void updateControlsForSetPoint(double[] currentPos, double[] target, boolean isShoot, TerraBot bot){
        setpointController.update(currentPos, target);

        if (setpointController.isDone()) {
            if(!isShoot) {
                next();
            }else{
                bot.autoAimer.reached();
            }
        }


    }

    public void resetControls(){
        waypointController.reset();
        setpointController.reset();
    }


    public double[] update(double[] currentPos, TerraBot bot){
        double[] target = poses.get(curIndex+1);
        switch (posetypes.get(curIndex+1)){
            case WAYPOINT:
                updateControlsForWaypoint(currentPos, target, lines.get(curIndex));
                return waypointController.getPowers();
            case SETPOINT:
                updateControlsForSetPoint(currentPos, target, false, bot);
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
                updateControlsForSetPoint(currentPos, targetSh, true, bot);
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


    public void start(TerraBot bot, LinearOpMode op){
        op.telemetry.addData("Resetting", "Odometry");
        op.telemetry.update();
        telemetryHandler.init(op.telemetry, bot);
        bot.odometry.resetAll(poses.get(0));
        bot.angularPosition.resetGyro(poses.get(0)[2]);
        op.telemetry.addData("Starting", "Movement");
        op.telemetry.update();
        globalTime.reset();
        while (op.opModeIsActive() && isExecuting) {
            double[] pows = update(bot.odometry.getAll(), bot);
            bot.move(pows[1], pows[0], pows[2]);
//            Sleep.trySleep(() -> Thread.sleep(10));
        }
        op.telemetry.addData("Status:", "Done");
        op.telemetry.update();
        bot.move(0,0,0);
        stopRFThread();
    }

    public enum Posetype{
        WAYPOINT,
        SETPOINT,
        STOP,
        SHOOT
    }

}