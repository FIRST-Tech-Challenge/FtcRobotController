package autofunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

import autofunctions.RobotFunctionsHandler;
import developing.MotionPlanner;
import global.TerraBot;
import globalfunctions.Optimizer;
import globalfunctions.PID;
import globalfunctions.Sleep;
import globalfunctions.Storage;
import globalfunctions.TelemetryHandler;
import globalfunctions.TimeData;
import util.CodeSeg;
import util.Line;
import util.Vector;

public class Path {

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

    final public double[] fs = {0.5,0.5, 100}; // {0.15,0.15,0.27}; // m/s^2, m/s^2, deg/s^2
    final public double[] ms = {8,9.4,2500}; // m/s^2, m/s^2, deg/s^2
    final public double[] ls = {0.35,0.35,0.35}; //s, s, s

    public double xp = 0.005;
    public double yp = 0.005;
    public double hp = 0.0;

    //Accs in meters
    public double XAcc = 0.01;
    public double YAcc = 0.01;
    public double HAcc = 1;

    public Storage storage = new Storage();
    public ArrayList<double[]> track = new ArrayList<>();
    public ArrayList<double[]> targets = new ArrayList<>();
    public ArrayList<Double> trackTimes = new ArrayList<>();
    public ArrayList<double[]> speeds = new ArrayList<>();
    public ElapsedTime trackTime = new ElapsedTime();

    public boolean globalMode = false;

    public double angleToGoal = 0;

    public double lxpos = 0;
    public double lypos = 0;
    public double lhpos = 0;

    public double lxvel = 0;
    public double lyvel = 0;
    public double lhvel = 0;

    public double xerr = 0;
    public double yerr = 0;
    public double herr = 0;

    public boolean xdone = false;
    public boolean ydone = false;
    public boolean hdone = false;

    public double ltime = 0;

    public int updateNum = 0;

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
        xControl.setCoefficients(ks[0], ds[0], is[0]);
        yControl.setCoefficients(ks[1], ds[1], is[1]);
        hControl.setCoefficients(ks[2], ds[2], is[2]);
        xControl.setMaxD(1);
        yControl.setMaxD(1);
        hControl.setMaxD(1);
        xMP.setFML(fs[0], ms[0], ls[0]);
        yMP.setFML(fs[1], ms[1], ls[1]);
        hMP.setFML(fs[2], ms[2], ls[2]);
        xMP.setAcc(XAcc);
        yMP.setAcc(YAcc);
        hMP.setAcc(HAcc);
        xMP.setAclP(xp);
        yMP.setAclP(yp);
        hMP.setAclP(hp);
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
        updateNum = 0;
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

    public void updateControlsForSetPoint(double[] currentPos, double[] target){
        xerr = (target[0]-currentPos[0])/100;
        yerr = (target[1]-currentPos[1])/100;

        Vector errVect = new Vector(xerr,yerr);
        errVect.rotate(-currentPos[2], Vector.angle.DEGREES);

        xerr = errVect.x;
        yerr = errVect.y;

        herr = target[2] - currentPos[2];
        double deltaX = currentPos[0]/100 - lxpos;
        double deltaY = currentPos[1]/100 - lypos;
        double deltaH = currentPos[2]- lhpos;
        lxpos = currentPos[0]/100;
        lypos = currentPos[1]/100;
        lhpos = currentPos[2];

        double curTime = globalTime.seconds();
        double deltaT = curTime-ltime;
        ltime = curTime;

        double xvel = deltaX/deltaT;
        double yvel = deltaY/deltaT;
        double hvel = deltaH/deltaT;

        double deltaXV = xvel-lxvel;
        double deltaYV = yvel-lyvel;
        double deltaHV = hvel-lhvel;

        lxvel = xvel;
        lyvel = yvel;
        lhvel = hvel;

        double xacl = deltaXV/deltaT;
        double yacl = deltaYV/deltaT;
        double hacl = deltaHV/deltaT;



        if(updateNum > 1){
            xMP.update(xerr, xvel, xacl);
            yMP.update(yerr, yvel, yacl);
            hMP.update(herr, hvel, hacl);
        }else{
            updateNum++;
        }

        if(hasReachedSetpoint(xerr, yerr, herr)){
            next();
        }

    }

    public void resetControls(){
        xControl.reset();
        yControl.reset();
        hControl.reset();
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
                updateControlsForSetPoint(currentPos, targetS);
                return calcPows(false);
            case STOP:
                if(globalTime.seconds() > stops.get(stopIndex)){
                    next();
                    stopIndex++;
                }
                return new double[]{0,0,0};
            case SHOOT:
                double[] targetSh = poses.get(curIndex+1);
                targetSh = new double[]{targetSh[0], targetSh[1], angleToGoal};
                updateControlsForSetPoint(currentPos, targetSh);
                if(!bot.autoAimer.hasPosBeenUpdated()) {
                    bot.autoAimer.setOuttakePos(Arrays.copyOf(targetSh, 2));
                    angleToGoal = bot.autoAimer.getRobotToGoalAngle(targetSh);
                }
                if(bot.autoAimer.isDone){
                    bot.autoAimer.ready();
                    next();
                }
                bot.outtakeWithCalculations(false);
                return calcPows(false);
            default:
                return new double[]{0,0,0};
        }



    }

    public boolean hasReachedSetpoint(double xerr, double yerr, double herr){
        xdone = xMP.isDone(xerr);
        ydone = yMP.isDone(yerr);
        hdone = hMP.isDone(herr);
        return xdone && ydone && hdone;
    }

    public double[] calcPows(boolean isWay){
        if (isWay) {
            double[] out = new double[3];
            out[0] = -Range.clip(xControl.getPower(), -1, 1);
            out[1] = -Range.clip(yControl.getPower(), -1, 1);
            out[2] = -Range.clip(hControl.getPower(), -1, 1);
            return out;
        }else{
            double[] out = new double[3];
            out[0] = Range.clip(xMP.getPower(), -0.5, 0.5);
            out[1] = Range.clip(yMP.getPower(), -0.5, 0.5);
            out[2] = Range.clip(hMP.getPower(), -0.5, 0.5);
            return out;
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
//
//            op.telemetry.addData("xerr", xerr);
//            op.telemetry.addData("yerr", yerr);
//            op.telemetry.addData("herr", herr);
//            op.telemetry.addData("xdone", xdone);
//            op.telemetry.addData("ydone", ydone);
//            op.telemetry.addData("hdone", hdone);
//            op.telemetry.update();
//
//            Sleep.trySleep(() -> Thread.sleep(10));
        }
        op.telemetry.addData("Status:", "Done");
        op.telemetry.update();
        bot.move(0,0,0);
        stopRFThread();
    }

    public void saveData(){
        storage.saveTimeData(new TimeData("Current", track, trackTimes));
        storage.saveTimeData(new TimeData("Current2", speeds, trackTimes));
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