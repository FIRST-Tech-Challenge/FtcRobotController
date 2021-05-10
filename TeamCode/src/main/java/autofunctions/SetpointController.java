package autofunctions;


import com.qualcomm.robotcore.util.Range;

import autofunctions.MotionPlanner;
import util.Vector;

public class SetpointController {
    //Ps are proportional to the difference in target and current velocities
    //As define the shape of the v vs s plot, keep in range (0,1]
    //Lower value of a means it slows down quicker higher means that it stays moving for longer
    //Rs are the rest powers
    final public double[] ps = {0.01,0.005, 0.001};
    final public double[] as = {0.5,0.5,0.5};
    final public double[] rs = {0.15,0.07,0.1};

    //X, Y, and H accuracies
    public double XAcc = 1; //cm
    public double YAcc = 1; //cm
    public double HAcc = 1; //deg
    //X, Y, and H motion planners
    public MotionPlanner xMP = new MotionPlanner();
    public MotionPlanner yMP = new MotionPlanner();
    public MotionPlanner hMP = new MotionPlanner();

    //Initialize the motion planners with PAR and accs
    public void init(){
        xMP.setPAR(ps[0], as[0], rs[0]);
        yMP.setPAR(ps[1], as[1], rs[1]);
        hMP.setPAR(ps[2], as[2], rs[2]);
        xMP.setAcc(XAcc);
        yMP.setAcc(YAcc);
        hMP.setAcc(HAcc);
    }
    //Updates the motion planners with the current position and target
    public void update(double[] currentPos, double[] target){
        //If the starting position has been set then update
        if(xMP.hasStartDisBeenSet && yMP.hasStartDisBeenSet && hMP.hasStartDisBeenSet){
            //Current position vector
            Vector curPosVect = new Vector(currentPos[0] , currentPos[1]);
            curPosVect.rotate(-currentPos[2], Vector.angle.DEGREES);
            //Start position vector
            Vector startVect = new Vector(xMP.startDis, yMP.startDis);
            startVect.rotate(-currentPos[2], Vector.angle.DEGREES);
            //Target vector
            Vector targetVect = new Vector(target[0], target[1]);
            targetVect.rotate(-currentPos[2], Vector.angle.DEGREES);

            //Local distance is the current pos - the start pos
            Vector localDis = curPosVect.subtract(startVect);
            //Local target is the target pos - the start pos
            Vector localTarget = targetVect.subtract(startVect);
            //Update the motion planners with the x and y coords
            xMP.update(localDis.x, localTarget.x);
            yMP.update(localDis.y, localTarget.y);
            hMP.update(currentPos[2]-hMP.startDis, target[2]-hMP.startDis);
        }else{
            //Set the starting positions
            xMP.update(currentPos[0], target[0]);
            yMP.update(currentPos[1], target[1]);
            hMP.update(currentPos[2], target[2]);
        }

    }
    //Has the robot reached the target position
    public boolean isDone(){
        return xMP.isDone() && yMP.isDone() && hMP.isDone();
    }
    //Get the powers for the motors
    public double[] getPowers(){
        return new double[]{Range.clip(xMP.getPower(), -1, 1), Range.clip(yMP.getPower(), -1, 1), Range.clip(hMP.getPower(), -1, 1)};
    }
    //Reset the motion planners
    public void reset(){
        xMP.reset();
        yMP.reset();
        hMP.reset();
    }


}
