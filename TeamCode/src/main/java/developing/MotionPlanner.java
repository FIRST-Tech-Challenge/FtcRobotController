package developing;

public class MotionPlanner {
    //Distance to target
    public double startDis = 0;

    public double startTime = 0;
    //Starting velocity
    public double startVel = 0;
    //a coeff
    public double a = 0;
    //b coeff
    public double b = 0;
    //c coeff
    public double c = 0;

    //time A when half way
    public double tA = 0;
    //Time b when reached
    public double tB = 0;
    //Sc
    public double switchOverPoint = 0.8;


    //Orbital max torques = 1.23 Nm
    //Four motors = 4.94 Nm
    //Radius of wheels = 0.05 m
    //Force from wheels = 98.86 N
    //Mass of robot = 10.5 kg
    //Max Accel = 9.4 m/s^2
    public double maxAccel = 2; // m/s^2

    public double curPow = 0;

    //Weight required to pull = 2.5 kg
    //Force required to pull = 24.5 N
    //Max Friction Accel = 2.33
//    public double frictionAccel = 0.2; // m/s^2

//    public double frictionAccel = 0.2;

    //Friction Accel
    public double frictionAccel = 0.1;

    //Slow Range
    public double slowRange = 0.03;

    public double Acc = 0.005;

    public boolean hasTargetBeenSet = false;



    public void setTarget(double dis, double startVel, double startTime){
        this.startDis = dis;
        this.startVel = startVel;
        this.startTime = startTime;
        this.tA = -((2*startVel)-Math.sqrt(2*(2*Math.pow(startVel,2)+(3*maxAccel*dis)))/(maxAccel));
        calcABCs(dis, startVel, tA);
        hasTargetBeenSet = true;
    }

    public void setFMS(double fric, double maxAccel, double slowRange){
        this.frictionAccel = fric;
        this.maxAccel = maxAccel;
        this.slowRange = slowRange;
    }

    public void calcABCs(double d, double v, double t){
        this.a = -(6*d)/Math.pow(t,3) + (3*v)/(Math.pow(t,2));
        this.b = (6*d)/(Math.pow(t,2)) - (4*v)/t;
        this.c = v;
    }

    public double calcAccel(double curDis, double curVel, double curTime){
        if((curDis/startDis) > (1-switchOverPoint)){
            return considerFric(calcAccelA(curTime-startTime), curDis);
        }else{
            return considerFric(calcAccelB(curDis, curVel), curDis);
        }
    }

    public double calcAccelA(double curTime){
        return (2*a*curTime) + b;
    }

    public double calcAccelB(double curDis, double curVel){
        if(Math.abs(curDis) > slowRange) {
            tB = Math.abs((3 * curDis) / curVel);
        }else{
            tB = Math.abs((3 * 0.1) / curVel);
        }
        calcABCs(curDis, curVel, tB);
        return b;
    }


    public double considerFric(double accel, double dis){
        return accel + (Math.signum(dis)*frictionAccel);
    }

    public void update(double curDis, double curVel, double curTime){
        double accel = calcAccel(curDis, curVel, curTime);
        curPow = (accel/maxAccel);
    }

    public double getPower(){
        return curPow;
    }

    public void reset(){
        hasTargetBeenSet = false;
    }


    public boolean isDone(double dis){
        return Math.abs(dis) < Acc;
    }

    public void setAcc(double acc){
        Acc = acc;
    }







}
