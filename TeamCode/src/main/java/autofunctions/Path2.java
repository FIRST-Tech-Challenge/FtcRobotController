package autofunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

import developing.MotionPlanner;
import global.TerraBot;
import globalfunctions.PID;
import globalfunctions.Storage;
import globalfunctions.TelemetryHandler;
import globalfunctions.TimeData;
import util.CodeSeg;
import util.Line;
import util.Vector;

public class Path2 {

    public PID xControl = new PID();
    public PID yControl = new PID();
    public PID hControl = new PID();

    public MotionPlanner xMP = new MotionPlanner();
    public MotionPlanner yMP = new MotionPlanner();
    public MotionPlanner hMP = new MotionPlanner();

    public ElapsedTime globalTime = new ElapsedTime();
    public ElapsedTime endTimer = new ElapsedTime();

    public double maxRadius = 15;
    public double radius = 5;
    public double t = 0;
    public double ans = 0;
    public int curIndex = 0;


    public double[] targetPos = {0,0};

    public ArrayList<double[]> poses = new ArrayList<>();
    public ArrayList<Line> lines = new ArrayList<>();
    public ArrayList<Posetype> posetypes = new ArrayList<>();
    public RobotFunctionsHandler rfsHandler = new RobotFunctionsHandler();
    public RobotFunctionsHandler wobbleGoalHandler = new RobotFunctionsHandler();

    public ArrayList<Double> stops = new ArrayList<>();
    public int stopIndex = 0;
    public TelemetryHandler telemetryHandler = new TelemetryHandler();

    public boolean isExecuting = true;

    final public double[] ks = {0.05,0.05,0.012};
    final public double[] ds = {0.0003,0.0006,0.0005};
    final public double[] is = {0.000,0.000,0.0000};

    final public double[] fs = {0.1,0.1,0.1}; // m/s^2
    final public double[] ms = {2,2,2}; // m/s^2
    final public double[] ss = {0.03,0.03,0.03}; //m

    //Accs in meters
    public double XAcc = 0.5/100;
    public double YAcc = 0.5/100;
    public double HAcc = 0.5/100;

    public Storage storage = new Storage();
    public ArrayList<double[]> track = new ArrayList<>();
    public ArrayList<double[]> targets = new ArrayList<>();
    public ArrayList<Double> trackTimes = new ArrayList<>();
    public ArrayList<double[]> speeds = new ArrayList<>();
    public ElapsedTime trackTime = new ElapsedTime();

    public boolean globalMode = false;

    public double angleToGoal = 0;

    public double lxerr = 0;
    public double lyerr = 0;
    public double lherr = 0;

    public double ltime = 0;


    public Path2(double sx, double sy, double sh){
        poses.add(new double[]{sx, sy, sh});
        init();
    }
    public void init(){
        posetypes.add(Posetype.SETPOINT);
        xControl.setCoefficients(ks[0], ds[0], is[0]);
        yControl.setCoefficients(ks[1], ds[1], is[1]);
        hControl.setCoefficients(ks[2], ds[2], is[2]);
        xControl.setMaxD(1);
        yControl.setMaxD(1);
        hControl.setMaxD(1);
        xMP.setFMS(fs[0], ms[0], ss[0]);
        yMP.setFMS(fs[1], ms[1], ss[1]);
        hMP.setFMS(fs[2], ms[2], ss[2]);
        xMP.setAcc(XAcc);
        yMP.setAcc(YAcc);
        hMP.setAcc(HAcc);
        globalTime.reset();
        addStop(0.01);
    }

    public void updateRadius(double dis){ radius = maxRadius*(1-Math.exp(-(1/maxRadius)*(dis))); }
    public void setGlobalMode(boolean val){
        globalMode = val;
    }
    public void addNewPose(double x, double y, double h){
        if(!globalMode) {
            double[] lastPose = poses.get(poses.size() - 1);
            poses.add(new double[]{lastPose[0] + x, lastPose[1] + y, lastPose[2] + h});
            double[] currPose = poses.get(poses.size() - 1);
            lines.add(new Line(lastPose[0], lastPose[1], currPose[0], currPose[1]));
        }else{
            double[] lastPose = poses.get(poses.size() - 1);
            poses.add(new double[]{x, y, h});
            double[] currPose = poses.get(poses.size() - 1);
            lines.add(new Line(lastPose[0], lastPose[1], currPose[0], currPose[1]));
        }
    }
    public void addWaypoint(double x, double y, double h){
        addNewPose(x,y,h);
        posetypes.add(Posetype.WAYPOINT);
        rfsHandler.notRF();
        wobbleGoalHandler.notRF();
    }
    public void addSetpoint(double x, double y, double h){
        addNewPose(x,y,h);
        posetypes.add(Posetype.SETPOINT);
        rfsHandler.notRF();
        wobbleGoalHandler.notRF();
    }

    public void addStop(double time){
        addNewPose(0,0,0);
        posetypes.add(Posetype.STOP);
        stops.add(time);
        rfsHandler.notRF();
        wobbleGoalHandler.notRF();
    }


    public void addShoot(double x, double y, TerraBot bot){
        double[] last = Arrays.copyOf(poses.get(poses.size()-1), 2);
        last[0] += x;
        last[1] += y;
        angleToGoal = bot.autoAimer.getRobotToGoalAngle(last);
        addNewPose(x, y, 0);
        posetypes.add(Posetype.SHOOT);
        rfsHandler.notRF();
        wobbleGoalHandler.notRF();
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
        resetMPs();
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

    public double solve(double[] currentPose){
        double x1 = lines.get(curIndex).x1;
        double y1 = lines.get(curIndex).y1;
        double mx = lines.get(curIndex).mx;
        double my = lines.get(curIndex).my;
        double xr = currentPose[0];
        double yr = currentPose[1];
        double dx = x1-xr;
        double dy = y1-yr;
        double a = (mx*mx)+(my*my);
        double b = 2*((dx*mx)+(dy*my));
        double c = (dx*dx)+(dy*dy)-(radius*radius);
        double disc = (b * b) - (4 * a * c);
        ans = (-1)*((b - Math.sqrt(disc)) / (2 * a));
        if(!Double.isNaN(ans)) {
            if(ans > 0.99){
                next();
                return 1;
            }else {
                return ans;
            }
        }else{
            return 0;
        }
    }

    public double[] getTargetPos(double[] currentPose){
        t = solve(currentPose);
        targetPos = lines.get(curIndex).getAt(t);
        return targetPos;
    }


    public void updateControlsForWaypoint(double[] currentPos, double[] target){
        double xdis = currentPos[0] - target[0];
        double ydis = currentPos[1] - target[1];
        double herr = currentPos[2] - poses.get(curIndex + 1)[2];
        Vector disVect = new Vector(xdis,ydis);
        disVect.rotate(-currentPos[2], Vector.angle.DEGREES);
        xControl.update(disVect.x);
        yControl.update(disVect.y);
        hControl.update(herr);
    }

    public void updateControlsForSetPoint(double[] currentPos, double[] target, double curTime){

        double xerr = target[0] - currentPos[0];
        double yerr = target[1]- currentPos[1];
        double herr = target[2] - currentPos[2];
        double deltaX = lxerr - xerr;
        double deltaY = lyerr - yerr;
        double deltaH = lherr - herr;
        lxerr = xerr;
        lyerr = yerr;
        lherr = herr;

        double deltaT = curTime-ltime;
        ltime = curTime;

        double xvel = deltaX/deltaT;
        double yvel = deltaY/deltaT;
        double hvel = deltaH/deltaT;

        if(!xMP.hasTargetBeenSet && Math.abs(deltaX) < 2){
            xMP.setTarget(xerr/100, xvel/100, curTime);
            xMP.curPow = 0;
        }else{
            xMP.update(xerr, xvel, curTime);
        }
        if(!yMP.hasTargetBeenSet && Math.abs(deltaY) < 2){
            yMP.setTarget(yerr/100, yvel/100, curTime);
            yMP.curPow = 0;
        }else{
            yMP.update(yerr, yvel, curTime);
        }
        if(!hMP.hasTargetBeenSet && Math.abs(deltaH) < 2){
            hMP.setTarget(herr/100, hvel/100, curTime);
            hMP.curPow = 0;
        }else{
            hMP.update(herr, hvel, curTime);
        }

        hasReachedSetpoint(xerr, yerr, herr);
    }

    public void resetControls(){
        xControl.reset();
        yControl.reset();
        hControl.reset();
    }

    public void resetMPs(){
        xMP.reset();
        yMP.reset();
        hMP.reset();
    }

    public void scaleControls(double scale){
        xControl.scaleCoeffs(scale);
        yControl.scaleCoeffs(scale);
        hControl.scaleCoeffs(scale);
    }

    public double[] update(double[] currentPos, TerraBot bot){
        switch (posetypes.get(curIndex+1)){
            case WAYPOINT:
                double[] targetW = getTargetPos(currentPos);
                targets.add(targetW);
                updateControlsForWaypoint(currentPos, targetW);
                updateRadius(lines.get(curIndex).getDis());
                return calcPows(true);
            case SETPOINT:
                double[] targetS = poses.get(curIndex+1);
                updateControlsForSetPoint(currentPos, targetS, globalTime.seconds());

                return calcPows(false);
            case STOP:
                if(globalTime.seconds() > stops.get(stopIndex)){
                    next();
                    stopIndex++;
                }
                return new double[]{0,0,0};
            case SHOOT:
                //Fix
                double[] target2 = poses.get(curIndex+1);
                if(!bot.autoAimer.hasPosBeenUpdated()) {
                    bot.autoAimer.setOuttakePos(Arrays.copyOf(target2, 2));
                }

                if(bot.autoAimer.isDone){
                    bot.autoAimer.ready();
                    next();
                }
                bot.outtakeWithCalculations(false);
//                if(xControl.isDone() && yControl.isDone() && hControl.isDone()){
//                    if(endTimer.seconds() > endWait) {
//                        bot.autoAimer.reached();
//                        return new double[]{0, 0, 0};
//                    }else{
//                        return calcPows();
//                    }
//                }else {
//                    endTimer.reset();
//                    return calcPows();
//                }

            default:
                return new double[]{0,0,0};
        }



    }

    public boolean hasReachedSetpoint(double xerr, double yerr, double herr){
        if(xMP.isDone(xerr) && yMP.isDone(yerr) && hMP.isDone(herr)){
            return true;
        }else{
            return false;
        }
    }

    public double[] calcPows(boolean isWay){
        if(isWay) {
            double[] out = new double[3];
            out[0] = -Range.clip(xControl.getPower(), -1, 1);
            out[1] = -Range.clip(yControl.getPower(), -1, 1);
            out[2] = -Range.clip(hControl.getPower(), -1, 1);
            return out;
        }else{
            double[] out = new double[3];
            out[0] = Range.clip(xMP.getPower(), -1, 1);
            out[1] = Range.clip(yMP.getPower(), -1, 1);
            out[2] = Range.clip(hMP.getPower(), -1, 1);
            return out;
        }
    }


    public void start(TerraBot bot, LinearOpMode op){
        op.telemetry.addData("Starting", "RF Threads");
        op.telemetry.update();
        telemetryHandler.init(op.telemetry, bot);
        bot.odometry.resetAll(poses.get(0));
        bot.angularPosition.resetGyro(poses.get(0)[2]);
        globalTime.reset();
        while (op.opModeIsActive() && isExecuting) {
            double[] pows = update(bot.odometry.getAll(), bot);
            bot.move(pows[1], pows[0], pows[2]);
            op.telemetry.update();
//            track.add(bot.odometry.getAll());
//            trackTimes.add(trackTime.seconds());

        }
        op.telemetry.addData("Status:", "Done");
        op.telemetry.update();
        bot.move(0,0,0);
        stopRFThread();
    }

    public void saveData(){
        storage.saveTimeData(new TimeData("Current", track, trackTimes));
//        storage.saveTimeData(new TimeData("Current2", speeds, trackTimes));
        storage.saveTimeData(new TimeData("Current3", poses, false));
        storage.saveTimeData(new TimeData("Current4", targets, false));
    }

    public enum Posetype{
        WAYPOINT,
        SETPOINT,
        STOP,
        SHOOT
    }

}