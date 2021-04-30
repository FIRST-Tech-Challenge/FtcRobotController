package autofunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;

import globalfunctions.Storage;
import globalfunctions.TelemetryHandler;
import global.TerraBot;
import globalfunctions.TimeData;
import util.CodeSeg;
import util.Line;
import globalfunctions.PID;
import util.Vector;

public class Path {

    public PID xControl = new PID();
    public PID yControl = new PID();
    public PID hControl = new PID();

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

    public double[] ksS = {0.02,0.01,0.012};
    public double[] dsS = {0.0003,0.0008,0.0005};
//    final public double[] isS = {0.0,0.0,0.0};

    public double[] isS = {0.1,0.08,0.1};

    public double xRestPow = 0.00;
    public double yRestPow = 0.00;
    public double hRestPow = 0.02;



    public double XAcc = 1;
    public double YAcc = 1;
    public double HAcc = 1;


    final public double endWait = 0.1; // 0.2

    public Storage storage = new Storage();
    public ArrayList<double[]> track = new ArrayList<>();
    public ArrayList<double[]> targets = new ArrayList<>();
    public ArrayList<Double> trackTimes = new ArrayList<>();

    public ArrayList<double[]> speeds = new ArrayList<>();

    public ElapsedTime trackTime = new ElapsedTime();

    public boolean globalMode = false;





    public Path(double sx, double sy, double sh){
        poses.add(new double[]{sx, sy, sh});
        init();
    }
    public Path(double[] pos){
        poses.add(pos);
        init();
    }
    public void init(){
        posetypes.add(Posetype.SETPOINT);
        setCoeffsForWaypoint();
        xControl.setAcc(XAcc);
        yControl.setAcc(YAcc);
        hControl.setAcc(HAcc);
        xControl.setRestPow(xRestPow);
        yControl.setRestPow(yRestPow);
        hControl.setRestPow(hRestPow);
        xControl.setMaxI(0.15);
        yControl.setMaxI(0.15);
        hControl.setMaxI(0.15);
        xControl.setMaxD(1);
        yControl.setMaxD(1); // 0.55
        hControl.setMaxD(1);
        xControl.setRangeI(5);
        yControl.setRangeI(5);
        hControl.setRangeI(5);
        globalTime.reset();
        addStop(0.01);
    }

    public void init2() {
        xControl.setAcc(XAcc);
        yControl.setAcc(YAcc);
        hControl.setAcc(HAcc);
        xControl.setRestPow(xRestPow);
        yControl.setRestPow(yRestPow);
        hControl.setRestPow(hRestPow);
        xControl.setMaxI(0.15);
        yControl.setMaxI(0.15);
        hControl.setMaxI(0.15);
        xControl.setMaxD(1);
        yControl.setMaxD(1); // 0.55
        hControl.setMaxD(1);
        xControl.setRangeI(5);
        yControl.setRangeI(5);
        hControl.setRangeI(5);
    }

    public void setCoeffsForSetpoint(){
        xControl.setCoefficients(ksS[0], dsS[0], isS[0]);
        yControl.setCoefficients(ksS[1], dsS[1], isS[1]);
        hControl.setCoefficients(ksS[2], dsS[2], isS[2]);
        xControl.scaleAccs(1);
        yControl.scaleAccs(1);
        hControl.scaleAccs(1);
    }
    public void setCoeffsForWaypoint(){
        xControl.setCoefficients(ks[0], ds[0], is[0]);
        yControl.setCoefficients(ks[1], ds[1], is[1]);
        hControl.setCoefficients(ks[2], ds[2], is[2]);
    }

    public void setCoeffsForShoot(){
        xControl.setCoefficients(ksS[0], dsS[0], isS[0]);
        yControl.setCoefficients(ksS[1], dsS[1], isS[1]);
        hControl.setCoefficients(ksS[2], dsS[2], isS[2]);
        xControl.scaleAccs(1);
        yControl.scaleAccs(1);
        hControl.scaleAccs(1);
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

    public void disableIs(){
        xControl.disableI = true;
        yControl.disableI = true;
        hControl.disableI = true;
    }

    public void addShoot(double x, double y, TerraBot bot){
        double[] last = Arrays.copyOf(poses.get(poses.size()-1), 2);
        last[0] += x;
        last[1] += y;
        addNewPose(x, y, bot.autoAimer.getRobotToGoalAngle(last));
        //LOOK HERE
        posetypes.add(Posetype.SHOOT);
        rfsHandler.notRF();
        wobbleGoalHandler.notRF();
        addSetpoint(0,0,-bot.autoAimer.getRobotToGoalAngle(last));
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


    public void updateControls(double[] currentPos, double[] target, boolean isShoot){
        double xdis = currentPos[0] - target[0];
        double ydis = currentPos[1] - target[1];
        double herr;
        if(!isShoot) {
            herr = currentPos[2] - poses.get(curIndex + 1)[2];
        }else{
            herr = currentPos[2] - target[2];
        }

        Vector disVect = new Vector(xdis,ydis);
        disVect.rotate(-currentPos[2], Vector.angle.DEGREES);

        xControl.update(disVect.x);
        yControl.update(disVect.y);
        hControl.update(herr);

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
                setCoeffsForWaypoint();
                double[] target = getTargetPos(currentPos);
                targets.add(target);
                updateControls(currentPos,target, false);
                updateRadius(lines.get(curIndex).getDis());
                return calcPows();
            case SETPOINT:
                setCoeffsForSetpoint();
                double[] target1 = poses.get(curIndex+1);
                updateControls(currentPos,target1, false);
                hasReachedSetpoint();
                return calcPows();
            case STOP:
                if(globalTime.seconds() > stops.get(stopIndex)){
                    next();
                    stopIndex++;
                }
                return new double[]{0,0,0};
            case SHOOT:
                setCoeffsForShoot();
                double[] target2 = poses.get(curIndex+1);
                if(!bot.autoAimer.hasPosBeenUpdated()) {
                    bot.autoAimer.setOuttakePos(Arrays.copyOf(target2, 2));
                }
                updateControls(currentPos, target2, true);

                if(bot.autoAimer.isDone){
                    bot.autoAimer.ready();
                    next();
                }
                bot.outtakeWithCalculations(false);
                if(xControl.isDone() && yControl.isDone() && hControl.isDone()){
                    if(endTimer.seconds() > endWait) {
                        bot.autoAimer.reached();
                        return new double[]{0, 0, 0};
                    }else{
                        return calcPows();
                    }
                }else {
                    endTimer.reset();
                    return calcPows();
                }
            default:
                return new double[]{0,0,0};
        }



    }

    public void hasReachedSetpoint(){
        if(xControl.isDone() && yControl.isDone() && hControl.isDone()){
            if(endTimer.seconds() > endWait) {
                next();
            }
        }else{
            endTimer.reset();
        }
    }

    public double[] calcPows(){
        double[] out = new double[3];
        out[0] = -Range.clip(xControl.getPower(),-1,1);
        out[1] = -Range.clip(yControl.getPower(),-1,1);
        out[2] = -Range.clip(hControl.getPower(),-1,1);
        return out;
    }

    public double[] normalize(double[] in){
        double sum = Math.abs(in[0]) + Math.abs(in[1]) + Math.abs(in[2]);
        if(sum > 1){
            return new double[]{in[0]/sum, in[1]/sum, in[2]/sum};
        }else{
            return in;
        }
    }


    public void start(TerraBot bot, LinearOpMode op){
        globalTime.reset();
        op.telemetry.addData("Starting", "RF Threads");
        op.telemetry.update();
        telemetryHandler.init(op.telemetry, bot);
        bot.odometry.resetAll(poses.get(0));
        bot.angularPosition.resetGyro(poses.get(0)[2]);
        trackTime.reset();
        while (op.opModeIsActive() && isExecuting) {
            double[] pows = update(bot.odometry.getAll(), bot);
            track.add(bot.odometry.getAll());
            trackTimes.add(trackTime.seconds());
            bot.move(pows[1], pows[0], pows[2]);
            telemetryHandler.addAuton(this, 1);
            op.telemetry.update();
        }
//        op.telemetry.addData("COMPLETED", Arrays.toString(bot.odometry.getPos()));
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