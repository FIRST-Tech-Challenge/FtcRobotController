package developing;


import com.qualcomm.robotcore.util.Range;

import util.Vector;

public class SetpointController {
    final public double[] ps = {0.01,0.005, 0.001};
    final public double[] as = {0.5,0.5,0.5};
    final public double[] rs = {0.15,0.07,0.1};

    public double XAcc = 1; //cm
    public double YAcc = 1; //cm
    public double HAcc = 1; //deg

    public MotionPlanner2 xMP = new MotionPlanner2();
    public MotionPlanner2 yMP = new MotionPlanner2();
    public MotionPlanner2 hMP = new MotionPlanner2();

    public void init(){
        xMP.setPAR(ps[0], as[0], rs[0]);
        yMP.setPAR(ps[1], as[1], rs[1]);
        hMP.setPAR(ps[2], as[2], rs[2]);
        xMP.setAcc(XAcc);
        yMP.setAcc(YAcc);
        hMP.setAcc(HAcc);
    }

    public void update(double[] currentPos, double[] target){

        if(xMP.hasStartDisBeenSet && yMP.hasStartDisBeenSet && hMP.hasStartDisBeenSet){
            Vector disVect = new Vector(currentPos[0] , currentPos[1]);
            disVect.rotate(-currentPos[2], Vector.angle.DEGREES);

            double targetx = target[0] - xMP.startDis;
            double targety = target[1] - yMP.startDis;
            double targeth = target[2] - hMP.startDis;

            Vector targetVect = new Vector(targetx, targety);
            targetVect.rotate(-currentPos[2], Vector.angle.DEGREES);
            xMP.update(disVect.x, targetVect.x);
            yMP.update(disVect.y, targetVect.y);
            hMP.update(currentPos[2], targeth);
        }else{
            xMP.update(currentPos[0], target[0]);
            yMP.update(currentPos[1], target[1]);
            hMP.update(currentPos[2], target[2]);
        }

    }


    public boolean isDone(){
        return xMP.isDone() && yMP.isDone() && hMP.isDone();
    }

    public double[] getPowers(){
        return new double[]{Range.clip(xMP.getPower(), -1, 1), Range.clip(yMP.getPower(), -1, 1), Range.clip(hMP.getPower(), -1, 1)};
    }

    public void reset(){
        xMP.reset();
        yMP.reset();
        hMP.reset();
    }


}
