package developing;

public class MotionPlanner {
    //a coeff
    public double a = 0;
    //b coeff
    public double b = 0;
    //c coeff
    public double c = 0;

    public double maxAccel = 2;

    public double curPow = 0;
    public double frictionAccel = 0.1;

    public double Acc = 0.005;

    public double lookAheadTime = 1;

    public void setFML(double fric, double maxAccel, double lookAheadTime){
        this.frictionAccel = fric;
        this.maxAccel = maxAccel;
        this.lookAheadTime = lookAheadTime;
    }

    public void calcABCs(double d, double v, double t){
        this.a = -(6*d)/Math.pow(t,3) + (3*v)/(Math.pow(t,2));
        this.b = (6*d)/(Math.pow(t,2)) - (4*v)/t;
        this.c = v;
    }

    public double calcAccel(double curDis, double curVel){
        return considerFric(calcAccelC(curDis, curVel), curVel);
    }

    public double calcAccelC(double curDis, double curVel){
        calcABCs(curDis, curVel, lookAheadTime);
        return b;
    }


    public double considerFric(double accel, double vel){
        return accel + (Math.signum(vel)*frictionAccel);
    }

    public void update(double curDis, double curVel){
        double accel = calcAccel(curDis, curVel);
        curPow = (accel/maxAccel);
    }

    public double getPower(){
        return curPow;
    }



    public boolean isDone(double dis){
        return Math.abs(dis) < Acc;
    }

    public void setAcc(double acc){
        Acc = acc;
    }







}
