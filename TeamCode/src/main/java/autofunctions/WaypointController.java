package autofunctions;

import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

import globalfunctions.PID;
import util.Line;
import util.Vector;

public class WaypointController {

    public PID xControl = new PID();
    public PID yControl = new PID();
    public PID hControl = new PID();


    final public double[] ks = {0.05,0.05,0.012};
    final public double[] ds = {0.0003,0.0006,0.0005};
    final public double[] is = {0.000,0.000,0.0000};



    public double maxRadius = 15;
    public double currentRadius = 5;


    public boolean isDone = false;


    public void init(){
        xControl.setCoefficients(ks[0], ds[0], is[0]);
        yControl.setCoefficients(ks[1], ds[1], is[1]);
        hControl.setCoefficients(ks[2], ds[2], is[2]);
        xControl.setMaxD(1);
        yControl.setMaxD(1);
        hControl.setMaxD(1);
    }

    public void updateRadius(double dis){currentRadius = maxRadius*(1-Math.exp(-(1/maxRadius)*(dis))); }

    public double solve(double[] currentPose,Line currentLine){
        double x1 = currentLine.x1;
        double y1 = currentLine.y1;
        double mx = currentLine.mx;
        double my = currentLine.my;
        double xr = currentPose[0];
        double yr = currentPose[1];
        double dx = x1-xr;
        double dy = y1-yr;
        double a = (mx*mx)+(my*my);
        double b = 2*((dx*mx)+(dy*my));
        double c = (dx*dx)+(dy*dy)-(currentRadius*currentRadius);
        double disc = (b * b) - (4 * a * c);
        double ans = (-1)*((b - Math.sqrt(disc)) / (2 * a));
        if(!Double.isNaN(ans)) {
            if(ans > 0.99){
                isDone = true;
                return 1;
            }else {
                return ans;
            }
        }else{
            return 1;
        }
    }

    public double[] getTargetPos(double[] currentPose, Line currentLine){
        return currentLine.getAt(solve(currentPose, currentLine));
    }


    public void update(double[] currentPos, double[] target, Line currentLine){
        double[] localTarget = getTargetPos(currentPos, currentLine);
        updateRadius(currentLine.getDis());
        double xerr = localTarget[0]-currentPos[0];
        double yerr = localTarget[1]-currentPos[1] ;
        double herr = target[2]-currentPos[2];
        Vector disVect = new Vector(xerr,yerr);
        disVect.rotate(-currentPos[2], Vector.angle.DEGREES);
        xControl.update(disVect.x);
        yControl.update(disVect.y);
        hControl.update(herr);
    }

    public double[] getPowers(){
        return new double[]{Range.clip(xControl.getPower(), -1, 1), Range.clip(yControl.getPower(), -1, 1), Range.clip(hControl.getPower(), -1, 1)};
    }

    public void reset(){
        xControl.reset();
        yControl.reset();
        hControl.reset();
        isDone = false;
    }


    public boolean isDone(){return isDone;}


}
