package autofunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import global.TerraBot;
import util.Line;
import util.PID;
import util.Vector;

public class Path {

    PID xControl = new PID();
    PID yControl = new PID();
    PID hControl = new PID();

    public double xerr = 0;
    public double yerr = 0;
    public double herr = 0;


    public double radius = 10;

    public int curIndex = 0;

    public ArrayList<double[]> poses = new ArrayList<>();
    public ArrayList<Line> lines = new ArrayList<>();

    public boolean isExecuting = true;

    public Path(double sx, double sy, double sh){
        poses.add(new double[]{sx, sy, sh});
        xControl.setCoeffecients(0.025, 0.0, 0.0);
        yControl.setCoeffecients(0.03, 0.0, 0.0);
        hControl.setCoeffecients(0.03, 0.0, 0.0);
    }



    public void addPose(double x, double y, double h){
        double[] lastPose = poses.get(poses.size()-1);
        poses.add(new double[]{lastPose[0]+x, lastPose[1]+y, lastPose[2]+h});
        double[] currPose = poses.get(poses.size()-1);
        double x1 = lastPose[0];
        double y1 = lastPose[1];
        double x2 = currPose[0];
        double y2 = currPose[1];
        lines.add(new Line(x1, y1, x2, y2));
    }

    public ArrayList<double[]> getAllPoses(){
        return poses;
    }

    public double solve(double[] currentPose){
        double ans = 0;
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
        ans = (-1)*((b - Math.sqrt((b * b) - (4 * a * c))) / (2 * a));

        if(!Double.isNaN(ans)) {
            if(ans > 1){
                curIndex++;
            }
            return ans;
        }else{
            return 0;
        }
    }


    public double[] getTargetPos(double[] currentPose){
        double t = solve(currentPose);
        if(curIndex >= lines.size()){
            isExecuting = false;
            curIndex--;
        }
        return lines.get(curIndex).getAt(t);
    }

    public double[] update(double[] currentPos){
        if(curIndex < (poses.size())){
            double[] target = getTargetPos(currentPos);
            xerr = currentPos[0]-target[0];
            yerr = currentPos[1]-target[1];
            herr = currentPos[2]-poses.get(curIndex+1)[2];
            return calcPows(currentPos);
        }else{
            return new double[]{0,0,0};
        }
    }

    public double[] calcPows(double[] currentPos){
        double robotTheta = currentPos[2];

        Vector mv = new Vector(xerr, yerr);
        mv = mv.getRotatedVec(robotTheta, Vector.angle.DEGREES);

        double[] out = new double[3];

        out[0] = -Math.signum(mv.x)* xControl.getPower(mv.x, 0,0);
        out[1] = -Math.signum(mv.y)* yControl.getPower(mv.y, 0,0);
        out[2] = -Math.signum(herr)*hControl.getPower(herr, 0,0);


        //for now fix later
        return out;
    }

    public void start(TerraBot bot, LinearOpMode op){
        while (op.opModeIsActive() && isExecuting){
            double[] pows = update(bot.odometry.getPos());
            bot.move(pows[1], pows[0], pows[2]);
            op.telemetry.addData("xpow",  pows[0]);
            op.telemetry.addData("ypow",  pows[1]);
            op.telemetry.addData("hpow",  pows[2]);
            op.telemetry.update();

//            double[] pows = path.update(bot.odometry.getPos());
//            bot.move(pows[1], pows[0], pows[2]);
        }
        bot.move(0,0,0);
    }


}