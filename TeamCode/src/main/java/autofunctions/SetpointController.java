package autofunctions;


import com.qualcomm.robotcore.util.Range;

import autofunctions.MotionPlanner;
import util.Vector;

public class SetpointController {
    final public double[] ps = {0.01,0.005, 0.001};
    final public double[] as = {0.5,0.5,0.5};
    final public double[] rs = {0.15,0.07,0.1};

    public double XAcc = 1; //cm
    public double YAcc = 1; //cm
    public double HAcc = 1; //deg

    public MotionPlanner xMP = new MotionPlanner();
    public MotionPlanner yMP = new MotionPlanner();
    public MotionPlanner hMP = new MotionPlanner();

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
            Vector curPosVect = new Vector(currentPos[0] , currentPos[1]);
            curPosVect.rotate(-currentPos[2], Vector.angle.DEGREES);

            Vector startVect = new Vector(xMP.startDis, yMP.startDis);
            startVect.rotate(-currentPos[2], Vector.angle.DEGREES);

            Vector targetVect = new Vector(target[0], target[1]);
            targetVect.rotate(-currentPos[2], Vector.angle.DEGREES);


            Vector localDis = curPosVect.subtract(startVect);
            Vector localTarget = targetVect.subtract(startVect);


            targetVect.rotate(-currentPos[2], Vector.angle.DEGREES);
            xMP.update(localDis.x, localTarget.x);
            yMP.update(localDis.y, localTarget.y);
            hMP.update(currentPos[2]-hMP.startDis, target[2]-hMP.startDis);
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
//        return new double[]{0,0,0};
    }

    public void reset(){
        xMP.reset();
        yMP.reset();
        hMP.reset();
    }


}
