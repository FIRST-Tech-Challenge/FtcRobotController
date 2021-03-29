package developing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

import global.TerraBot;
import util.CodeSeg;
import util.Line;
import util.PID;
import util.ThreadHandler;
import util.Vector;

public class Path2 {

    PID xControl = new PID();
    PID yControl = new PID();
    PID hControl = new PID();

    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime timer2 = new ElapsedTime();
    public ElapsedTime timer3 = new ElapsedTime();

    public double lastTime = -0.1;

    public double xerr = 0;
    public double yerr = 0;
    public double herr = 0;

    public double xder = 0;
    public double yder = 0;
    public double hder = 0;

    public double xint = 0;
    public double yint = 0;
    public double hint = 0;


    public double radius = 5;
    public double t = 0;
    public double ans = 0;
    public double ans2 = 0;
    public int curIndex = 0;

    public int rfsIndex = 0;
    public int stopIndex = 0;

    public int rfsQueueIndex = 0;



    public double dScale = 2;
    public double kScale = 1;


    public double[] targetPos = {0,0};

    public ArrayList<double[]> poses = new ArrayList<>();
    public ArrayList<Line> lines = new ArrayList<>();
    public ArrayList<Posetype> posetypes = new ArrayList<>();
    public ArrayList<CodeSeg> rfs = new ArrayList<>();
    public ArrayList<Boolean> isRf = new ArrayList<>();
    public ArrayList<Double> stops = new ArrayList<>();

    public ArrayList<CodeSeg> rfsQueue = new ArrayList<>();

    public ThreadHandler threadHandler = new ThreadHandler();

    public TelemetryHandler telemetryHandler = new TelemetryHandler();


    public boolean isExecuting = true;

    final public double[] ks = {0.04,0.03,0.02};
    final public double[] ds = {0.00015,0.00015,3};
    final public double[] is = {0.01,0.01,0.005};
    final public double XAccS = 1;
    final public double YAccS = 1;
    final public double HAccS = 2;

    public double XAcc = XAccS;
    public double YAcc = YAccS;
    public double HAcc = HAccS;
    final public double endWait = 0.3;
    public double restPowX = 0.1;
    public double restPowY = 0.05;
    public double restPowT = 0.3;

    public double shootSpeed = 1;








    public Path2(double sx, double sy, double sh){
        poses.add(new double[]{sx, sy, sh});
        posetypes.add(Posetype.SETPOINT);
        xControl.setCoefficients(ks[0], ds[0], is[0]);
        yControl.setCoefficients(ks[1], ds[1], is[1]);
        hControl.setCoefficients(ks[2], ds[2], is[2]);
        timer.reset();
    }

    public void updateRadius(double dis){
        if(dis < 25) {
            radius = dis * 0.5;
        }else{
            radius = 25;
        }
    }

    public void scaleKs(double sx, double sy, double sh){
        xControl.setCoefficients(ks[0]*sx, xControl.Kd, xControl.Ki);
        yControl.setCoefficients(ks[1]*sy, yControl.Kd, yControl.Ki);
        hControl.setCoefficients(ks[2]*sh, hControl.Kd, hControl.Ki);
    }
    public void scaleDs(double sx, double sy, double sh){
        xControl.setCoefficients(xControl.Kp, ds[0]*sx, xControl.Ki);
        yControl.setCoefficients(yControl.Kp, ds[1]*sy, yControl.Ki);
        hControl.setCoefficients(hControl.Kp, ds[2]*sh, hControl.Ki);
    }

    public void resetIs(){
        xint = 0;
        yint = 0;
        hint = 0;
    }

    public CodeSeg nullCode(){
        return new CodeSeg() {
            @Override
            public void run() {

            }
        };
    }



    public void addWaypoint(double x, double y, double h){
        double[] lastPose = poses.get(poses.size()-1);
        poses.add(new double[]{lastPose[0]+x, lastPose[1]+y, lastPose[2]+h});
        double[] currPose = poses.get(poses.size()-1);
        double x1 = lastPose[0];
        double y1 = lastPose[1];
        double x2 = currPose[0];
        double y2 = currPose[1];
        lines.add(new Line(x1, y1, x2, y2));
        posetypes.add(Posetype.WAYPOINT);
        rfs.add(nullCode());
        isRf.add(false);
    }

    public void addSetpoint(double x, double y, double h){
        double[] lastPose = poses.get(poses.size()-1);
        poses.add(new double[]{lastPose[0]+x, lastPose[1]+y, lastPose[2]+h});
        double[] currPose = poses.get(poses.size()-1);
        double x1 = lastPose[0];
        double y1 = lastPose[1];
        double x2 = currPose[0];
        double y2 = currPose[1];
        lines.add(new Line(x1, y1, x2, y2));
        posetypes.add(Posetype.SETPOINT);
        rfs.add(nullCode());
        isRf.add(false);
    }
    public void addStop(double time){
        double[] lastPose = poses.get(poses.size()-1);
        poses.add(new double[]{lastPose[0], lastPose[1], lastPose[2]});
        double[] currPose = poses.get(poses.size()-1);
        double x1 = lastPose[0];
        double y1 = lastPose[1];
        double x2 = currPose[0];
        double y2 = currPose[1];
        lines.add(new Line(x1, y1, x2, y2));
        posetypes.add(Posetype.STOP);
        rfs.add(nullCode());
        isRf.add(false);
        stops.add(time);
    }

    public void addRF(CodeSeg... segs){
        rfs.add(combineSegs(segs));
        isRf.add(true);
    }
    public CodeSeg combineSegs(final CodeSeg[] segs){
        return new CodeSeg() {
            @Override
            public void run() {
                for(CodeSeg seg:segs) {
                    seg.run();
                }
            }
        };
    }

    public void startRFThread(LinearOpMode op){
        threadHandler.startAutoThread(new CodeSeg() {
            @Override
            public void run() {
                if(rfsIndex < rfs.size()) {
                    if((rfsQueue.size()) > rfsQueueIndex){
                        rfsQueue.get(rfsQueueIndex).run();
                        rfsQueueIndex++;
                    }
                }
            }
        }, op, 100);
    }
    public void stopRFThread(){
        threadHandler.stopAutoThread();
    }

    public void next(){
        resetIs();
        timer.reset();
        timer3.reset();
        lastTime = -0.1;
        curIndex++;
        if(rfsIndex < (rfs.size()-1)) {
            if (isRf.get(rfsIndex+1)) {
                rfsQueue.add(rfs.get(rfsIndex+1));
                rfsIndex++;
            }
        }
        rfsIndex++;
        if(curIndex >= lines.size()){
            isExecuting = false;
            curIndex--;
        }
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
        ans2 = (-1)*((b + Math.sqrt(disc)) / (2 * a));

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

    public void updateDIs(double[] currentVels, boolean isSet){

        xder = currentVels[0];
        yder = currentVels[1];
        hder = currentVels[2];


        if(isSet) {
            if(Math.abs(herr) > HAcc || Math.abs(xerr) > XAcc || Math.abs(yerr)>YAcc) {
                if (Math.abs(hder) < 1 && Math.abs(xder) < 30 && Math.abs(yder) < 30) {
                    if(timer3.seconds() > 0.2) {
                        restPowT = 0.8;
                        restPowX = 0.3;
                        restPowY = 0.3;
                    }
                } else {
                    restPowT = 0.2;
                    restPowX = 0.1;
                    restPowY = 0.05;
                    timer3.reset();
                }
            }
            hint = 0;
            xint = 0;
            yint = 0;
            scaleDs(1, 1, dScale);
            scaleKs(kScale, kScale, 1.2);
        }else{
            scaleKs(kScale, kScale, 1.2);
            scaleDs(0.3, 0.3, 1);
            hint = 0;
            xint = 0;
            yint = 0;
        }
    }

    public double[] update(double[] currentPos, double[] currentVels){
        if(posetypes.get(curIndex+1).equals(Posetype.WAYPOINT)) {
            double[] target = getTargetPos(currentPos);
            xerr = currentPos[0] - target[0];
            yerr = currentPos[1] - target[1];
            herr = currentPos[2] - poses.get(curIndex + 1)[2];
            updateDIs(currentVels, false);
            updateRadius(lines.get(curIndex).getDis());
            return calcPows(currentPos,currentVels, false);
        }else if(posetypes.get(curIndex+1).equals(Posetype.SETPOINT)){
            double[] target = poses.get(curIndex+1);
            xerr = currentPos[0] - target[0];
            yerr = currentPos[1] - target[1];
            herr = currentPos[2] - target[2];
            updateDIs(currentVels, true);
            hasReachedSetpoint();
            return calcPows(currentPos,currentVels, true);
        }else
        if (posetypes.get(curIndex+1).equals(Posetype.STOP)){
            if(timer.seconds() > stops.get(stopIndex)){
                next();
                stopIndex++;
            }
            return new double[]{0,0,0};
        }else{
            return new double[]{0,0,0};
        }

    }
    public void hasReachedSetpoint(){
        boolean xIn = (Math.abs(xerr)) < XAcc;
        boolean yIn = (Math.abs(yerr)) < YAcc;
        boolean hIn = (Math.abs(herr)) < HAcc;
        if(xIn && yIn && hIn){
            if(timer2.seconds() > endWait) {
                next();
            }
        }else{
            timer2.reset();
        }
    }

    public double[] calcPows(double[] currentPos,double[] currentVels, boolean isSet){
        double robotTheta = currentPos[2];

        Vector mv = new Vector(xerr, yerr);
        mv = mv.getRotatedVec(-robotTheta, Vector.angle.DEGREES);
        Vector dv = new Vector(xder, yder);
        dv = dv.getRotatedVec(-robotTheta, Vector.angle.DEGREES);
        Vector iv = new Vector(xint, yint);
        iv = iv.getRotatedVec(-robotTheta, Vector.angle.DEGREES);

        double[] out = new double[3];

        out[0] = -Math.signum(mv.x) * xControl.getPower(mv.x, currentVels[0], iv.x);
        out[1] = -Math.signum(mv.y) * yControl.getPower(mv.y, currentVels[1], iv.y);
        out[2] = Math.signum(herr) * hControl.getPower(herr, currentVels[2], hint);

        out[0] = Range.clip(out[0], -1, 1);
        out[1] = Range.clip(out[1], -1, 1);
        out[2] = Range.clip(out[2], -1, 1);

        if(isSet) {
            out[0] += -Math.signum(mv.x) * restPowX;
            out[1] += -Math.signum(mv.y) * restPowY;
            out[2] += Math.signum(herr) * restPowT;

            out[0] = Range.clip(out[0], -1, 1);
            out[1] = Range.clip(out[1], -1, 1);
            out[2] = Range.clip(out[2], -1, 1);
        }

        return out;
    }

    public double[] normalize(double[] in){
        double sum = in[0]+in[1]+in[2];
        if(sum > 1){
            return new double[]{in[0]/sum, in[1]/sum, in[2]/(1.5*sum)};
        }else{
            return in;
        }

    }

    public void start(TestRobot bot, LinearOpMode op){
        timer.reset();
        op.telemetry.addData("Starting", "RF Threads");
        op.telemetry.update();
//        startRFThread(op);
        while (op.opModeIsActive() && isExecuting){
            op.telemetry = telemetryHandler.addAutoAimer(op.telemetry, bot);
            op.telemetry = telemetryHandler.addOuttake(op.telemetry, bot);
//            op.telemetry = telemetryHandler.addOdometry(op.telemetry, bot);
            op.telemetry.update();

            bot.outtakeWithCalculations();

//            double[] pows = update(bot.odometry.getPos(), bot.odometry.getVels()); //bot.odometry.getVels()
//            bot.move(pows[1], pows[0], pows[2]);
//            bot.updateOdometry();
        }
        op.telemetry.addData("COMPLETED", "");
        op.telemetry.update();
        bot.move(0,0,0);
        stopRFThread();
    }

    public enum Posetype{
        WAYPOINT,
        SETPOINT,
        STOP
    }

}