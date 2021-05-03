package developing;

public class MotionPlanner2 {

    public double restPow = 0;
    public double approachRate = 0;
    public double proportionalCoeff = 0;

    public double targetDis = 0;

    public double curPow = 0;


    public void setPAR(double p, double a, double r){
        restPow = p;
        approachRate = a;
        restPow = r;
    }



    public void setTargetDis(double td){
        targetDis = td;
    }


    public double VofS(double s){
        return approachRate*Math.pow(Math.abs(targetDis-s), 1/approachRate)*Math.signum(targetDis-s);
    }

    public double getRestPow(double curDis){
        return Math.signum(targetDis-curDis)*restPow;
    }

    public void update(double curDis, double curVel){
        double targetVel = VofS(curDis);
        curPow = (proportionalCoeff*(targetVel-curVel))+getRestPow(curDis);
    }

    public double getPower(){
        return curPow;
    }
}
