package autofunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.sql.Time;
import java.util.ArrayList;
import java.util.Arrays;

import globalfunctions.Constants;
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




    public double maxRadius = 25;
    public double radius = 5;
    public double t = 0;
    public double ans = 0;
    public int curIndex = 0;


    public double[] targetPos = {0,0};

    public ArrayList<double[]> poses = new ArrayList<>();
    public ArrayList<Line> lines = new ArrayList<>();
    public ArrayList<Posetype> posetypes = new ArrayList<>();
    public RobotFunctionsHandler rfsHandler = new RobotFunctionsHandler();

    public ArrayList<Double> stops = new ArrayList<>();
    public int stopIndex = 0;

    public TelemetryHandler telemetryHandler = new TelemetryHandler();


    public boolean isExecuting = true;

    final public double[] ks = {0.1,0.1,0.01};
    final public double[] ds = {0.01,0.015,0.001};
    final public double[] is = {0.00,0.00,0.0000};

    final public double[] ksS = {0.05,0.05,0.01};
    final public double[] dsS = {0.008,0.008,0.0005};
//    final public double[] isS = {0.0005,0.0005,0.00005};
    final public double[] isS = {0.000,0.000,0.0000};

    public double xRestPow = 0.05;
    public double yRestPow = 0.03;
    public double hRestPow = 0.05;



    public double XAcc = 1;
    public double YAcc = 1;
    public double HAcc = 0.5;


    final public double endWait = 0.2;

    public Storage storage = new Storage();
    public ArrayList<double[]> track = new ArrayList<>();
    public ArrayList<Double> trackTimes = new ArrayList<>();

    public ArrayList<double[]> speeds = new ArrayList<>();

    public ElapsedTime trackTime = new ElapsedTime();





    public Path(double sx, double sy, double sh){
        poses.add(new double[]{sx, sy, sh});
        posetypes.add(Posetype.SETPOINT);
        setCoeffsForWaypoint();
        xControl.setAcc(XAcc);
        yControl.setAcc(YAcc);
        hControl.setAcc(HAcc);
        xControl.setRestPow(xRestPow);
        yControl.setRestPow(yRestPow);
        hControl.setRestPow(hRestPow);
        xControl.setMaxI(0.05);
        yControl.setMaxI(0.05);
        hControl.setMaxI(0.05);
        globalTime.reset();
        addStop(0.01);
    }

    public void setCoeffsForSetpoint(double dis){
        xControl.setCoefficients(ksS[0], dsS[0], isS[0]);
        yControl.setCoefficients(ksS[1], dsS[1], isS[1]);
        hControl.setCoefficients(ksS[2], dsS[2], isS[2]);
        xControl.scaleCoeffs(40/dis);
        yControl.scaleCoeffs(40/dis);
    }
    public void setCoeffsForWaypoint(){
        xControl.setCoefficients(ks[0], ds[0], is[0]);
        yControl.setCoefficients(ks[1], ds[1], is[1]);
        hControl.setCoefficients(ks[2], ds[2], is[2]);
        xControl.scaleCoeffs(1);
        yControl.scaleCoeffs(1);
        hControl.scaleCoeffs(1);
    }

    public void setCoeffsForShoot(){
        xControl.setCoefficients(ksS[0], dsS[0], isS[0]);
        yControl.setCoefficients(ksS[1], dsS[1], isS[1]);
        hControl.setCoefficients(ksS[2], dsS[2], isS[2]);
        hControl.scaleCoeffs(2);
    }


    public void updateRadius(double dis){ radius = maxRadius*(1-Math.exp(-(1/maxRadius)*(dis))); }


    public void addNewPose(double x, double y, double h){
        double[] lastPose = poses.get(poses.size()-1);
        poses.add(new double[]{lastPose[0]+x, lastPose[1]+y, lastPose[2]+h});
        double[] currPose = poses.get(poses.size()-1);
        lines.add(new Line(lastPose[0], lastPose[1], currPose[0], currPose[1]));
    }


    public void addWaypoint(double x, double y, double h){
        addNewPose(x,y,h);
        posetypes.add(Posetype.WAYPOINT);
        rfsHandler.notRF();
    }

    public void addSetpoint(double x, double y, double h){
        addNewPose(x,y,h);
        posetypes.add(Posetype.SETPOINT);
        rfsHandler.notRF();
    }

    public void addStop(double time){
        addNewPose(0,0,0);
        posetypes.add(Posetype.STOP);
        stops.add(time);
        rfsHandler.notRF();
    }

    public void addShoot(){
        addNewPose(0,0,0);
        posetypes.add(Posetype.SHOOT);
        rfsHandler.notRF();
    }

    public void addRF(CodeSeg... segs){
        rfsHandler.addRFs(segs);
    }


    public void startRFThread(LinearOpMode op){
       rfsHandler.start(op);
    }
    public void stopRFThread(){
       rfsHandler.stop();
    }

    public void next(){
        resetControls();
        globalTime.reset();
        curIndex++;
        rfsHandler.update();
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
//        second answer = (-1)*((b + Math.sqrt(disc)) / (2 * a));
        if(!Double.isNaN(ans)) {
            if(ans > 1){
                next();
            }
            return ans;
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

    public double[] update(double[] currentPos, TerraBot bot){
        switch (posetypes.get(curIndex+1)){
            case WAYPOINT:
                setCoeffsForWaypoint();
                double[] target = getTargetPos(currentPos);
                updateControls(currentPos,target, false);
                updateRadius(lines.get(curIndex).getDis());
                return calcPows();
            case SETPOINT:
                setCoeffsForSetpoint(lines.get(curIndex).getDis());
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
                target2[2] = bot.autoAimer.getRobotToGoalAngle(bot.odometry.getPos());
                updateControls(currentPos,target2, true);
                if(!bot.outtaking){
                    next();
                }
                if(xControl.done() && yControl.done() && hControl.done()){
                    if((endTimer.seconds() > endWait)) {
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
        if(xControl.done() && yControl.done() && hControl.done()){
            if(endTimer.seconds() > endWait) {
                next();
            }
        }else{
            endTimer.reset();
        }
    }

    public double[] calcPows(){
        double[] out = new double[3];
        out[0] = -Range.clip(xControl.getPower(),-0.5,0.5);
        out[1] = -Range.clip(yControl.getPower(),-0.5,0.5);
        out[2] = -Range.clip(hControl.getPower(),-0.5,0.5);
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
        startRFThread(op);
        trackTime.reset();
        while (op.opModeIsActive() && isExecuting){
//            op.telemetry = telemetryHandler.addAutoAimer(op.telemetry, bot);
//            op.telemetry = telemetryHandler.addOuttake(op.telemetry, bot);
//            op.telemetry = telemetryHandler.addOdometry(op.telemetry, bot);
//
//            op.telemetry.addData("yControl", yControl.getPower());
//            op.telemetry.addData("error", hControl.error);
//            op.telemetry.addData("h", bot.odometry.h);
//            op.telemetry.addData("x", bot.odometry.x);
//            op.telemetry.addData("y", bot.odometry.y);
//            op.telemetry.addData("rfs index", rfsHandler.rfsIndex);
//
//            telemetryHandler.addAuton(this);
//            telemetryHandler.addAutoAimer();
//            op.telemetry = telemetryHandler.getTelemetry();
//            op.telemetry.addData("Odometry pos", Arrays.toString(bot.odometry.getPos())); 87, 71 FINE
//            op.telemetry.addData("Target speed L", bot.autoAimer.outlController.targetSpeed);
//            op.telemetry.addData("Target speed R", bot.autoAimer.outrController.targetSpeed);// 213.5 CORRECT
//            op.telemetry.addData("Outr speed", bot.autoAimer.outrController.currSpeed);
            op.telemetry.addData("targetPos", bot.autoAimer.outtakePos);
            op.telemetry.addData("oldtargetPos", bot.autoAimer.oldOuttakePos);
            op.telemetry.update();
//
            bot.outtakeWithCalculations();

            double[] pows = update(bot.odometry.getAll(), bot);
            track.add(bot.odometry.getAll());
            speeds.add(new double[] {bot.getRightAngVel(), bot.getLeftAngVel()});
            trackTimes.add(trackTime.seconds());
            bot.move(pows[1], pows[0], pows[2]);
        }
        op.telemetry.addData("COMPLETED", "");
        op.telemetry.update();
        bot.move(0,0,0);
        stopRFThread();
    }

    public void saveData(){
        TimeData timeData = new TimeData("Current", track, trackTimes);
        TimeData timeData2 = new TimeData("Current2", speeds, trackTimes);
        storage.saveText(storage.convertToJSON("Today", timeData), timeData.name);
        storage.saveText(storage.convertToJSON("Today", timeData2), timeData2.name);
    }

    public enum Posetype{
        WAYPOINT,
        SETPOINT,
        STOP,
        SHOOT
    }

}