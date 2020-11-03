package autofunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.android.dex.Code;

import java.util.ArrayList;

import global.TerraBot;
import util.CodeSeg;
import util.Line;
import util.PID;
import util.ThreadHandler;
import util.Vector;

public class Path {

    PID xControl = new PID();
    PID yControl = new PID();
    PID hControl = new PID();

    ElapsedTime timer = new ElapsedTime();

    public double lastTime = 0;

    public double xerr = 0;
    public double yerr = 0;
    public double herr = 0;

    public double lxerr = 0;
    public double lyerr = 0;
    public double lherr = 0;

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

    public double[] targetPos = {0,0};

    public ArrayList<double[]> poses = new ArrayList<>();
    public ArrayList<Line> lines = new ArrayList<>();
    public ArrayList<Posetype> posetypes = new ArrayList<>();
    public ArrayList<CodeSeg> rfs = new ArrayList<>();

    public ThreadHandler threadHandler = new ThreadHandler();


    public boolean isExecuting = true;
    public boolean runningRFs = true;

    final public double scale = 0.3;
    final public double[] ks = {0.05,0.05,0.01};
    final public double[] ds = {0.005, 0.005, 0.001};
    final public double[] is = {0.01,0.01,0.08};
    final public double XAcc = 1;
    final public double YAcc = 1;
    final public double HAcc = 2;
    final public double derWait = 0.5;



    public Path(double sx, double sy, double sh){
        poses.add(new double[]{sx, sy, sh});
        posetypes.add(Posetype.SETPOINT);
        xControl.setCoeffecients(ks[0], ds[0], is[0]);
        yControl.setCoeffecients(ks[1], ds[1], is[1]);
        hControl.setCoeffecients(ks[2], ds[2], is[2]);
        timer.reset();
    }

    public void updateScale(double dis){
        radius = dis*scale;
    }

    public void resetIs(){
        xint = 0;
        yint = 0;
        hint = 0;
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
        rfs.add(null);
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
        rfs.add(null);
    }

    public void addRF(CodeSeg seg){
        rfs.add(seg);
    }

    public void startRFThread(LinearOpMode op){
        threadHandler.startAutoThread(new CodeSeg() {
            @Override
            public void run() {
                if(rfsIndex < rfs.size()) {
                    if (rfs.get(rfsIndex) != null && runningRFs) {
                        rfs.get(rfsIndex).run();
                        rfsIndex++;
                    } else {
                        runningRFs = false;
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
        lastTime = 0;
        curIndex++;
        rfsIndex++;
        runningRFs = true;
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
                if(herr < (HAcc*6)) {
                    next();
                }else{
                    ans = 1;
                }
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

    public void updateDIs(){
        double changeT = timer.seconds() - lastTime;
        lastTime = timer.seconds();

        if(timer.seconds() > derWait) {
            xder = (xerr - lxerr) / changeT;
            yder = (yerr - lyerr) / changeT;
            hder = (herr - lherr) / changeT;
        }else {
            xder = (timer.seconds()/derWait) * (xerr - lxerr) / changeT;
            yder =  (timer.seconds()/derWait) * (yerr - lyerr) / changeT;
            hder =  (timer.seconds()/derWait) * (herr - lherr) / changeT;
        }
        lxerr = xerr;
        lyerr = yerr;
        lherr = herr;
        xint += xerr * changeT;
        yint += yerr * changeT;
        hint += herr * changeT;
    }

    public double[] update(double[] currentPos){
        if(posetypes.get(curIndex+1).equals(Posetype.WAYPOINT)) {
            double[] target = getTargetPos(currentPos);
            xerr = currentPos[0] - target[0];
            yerr = currentPos[1] - target[1];
            herr = currentPos[2] - poses.get(curIndex + 1)[2];
            updateDIs();
            updateScale(lines.get(curIndex).getDis());
        }else{
            double[] target = poses.get(curIndex+1);
            xerr = currentPos[0] - target[0];
            yerr = currentPos[1] - target[1];
            herr = currentPos[2] - target[2];
            updateDIs();
            hasReachedSetpoint();
        }
        return calcPows(currentPos);
    }
    public void hasReachedSetpoint(){
        boolean xIn = (Math.abs(xerr)) < XAcc;
        boolean yIn = (Math.abs(yerr)) < YAcc;
        boolean hIn = (Math.abs(herr)) < HAcc;
        if(xIn && yIn && hIn){
            next();
        }
    }

    public double[] calcPows(double[] currentPos){
        double robotTheta = currentPos[2];

        Vector mv = new Vector(xerr, yerr);
        mv = mv.getRotatedVec(-robotTheta, Vector.angle.DEGREES);
        Vector dv = new Vector(xder, yder);
        dv = dv.getRotatedVec(-robotTheta, Vector.angle.DEGREES);
        Vector iv = new Vector(xint, yint);
        iv = iv.getRotatedVec(-robotTheta, Vector.angle.DEGREES);

        double[] out = new double[3];

        out[0] = -Math.signum(mv.x) * xControl.getPower(mv.x, dv.x, iv.x);
        out[1] = -Math.signum(mv.y) * yControl.getPower(mv.y, dv.y, iv.y);
        out[2] = -Math.signum(herr) * hControl.getPower(herr, hder, hint);

        return normalize(out);
    }

    public double[] normalize(double[] in){
        double sum = in[0]+in[1]+in[2];
        if(sum > 1){
            return new double[]{in[0]/sum, in[1]/sum, in[2]/sum};
        }else{
            return in;
        }

    }

    public void start(TerraBot bot, LinearOpMode op){
        timer.reset();
        startRFThread(op);
        while (op.opModeIsActive() && isExecuting){
            double[] pows = update(bot.odometry.getPos());
            bot.move(pows[1], pows[0], pows[2]);
//            op.telemetry.addData("xpow",  pows[0]);
//            op.telemetry.addData("ypow",  pows[1]);
//            op.telemetry.addData("hpow",  pows[2]);
//            op.telemetry.addData("targetx", targetPos[0]);
//            op.telemetry.addData("targety", targetPos[1]);
//            op.telemetry.addData("t", t);
//            op.telemetry.addData("curInd", curIndex);
//            op.telemetry.addData("y1", lines.get(0).y1);
//            op.telemetry.addData("y2", lines.get(0).y2);
//            op.telemetry.addData("ans1", ans);
//            op.telemetry.addData("ans2", ans2);
//            op.telemetry.addData("x", bot.odometry.getX());
//            op.telemetry.addData("y", bot.odometry.getY());
//            op.telemetry.addData("herr", herr);
            op.telemetry.addData("y", yint);
            op.telemetry.update();
        }
        bot.move(0,0,0);
        stopRFThread();
    }

    public enum Posetype{
        WAYPOINT,
        SETPOINT;
    }


}