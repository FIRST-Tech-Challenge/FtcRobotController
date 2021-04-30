package developing;

public class MotionPlanner {
    public double dis = 0;
    public double startVel = 0;

    public double a = 0;
    public double b = 0;
    public double c = 0;

    public double tA = 0;
    public double tB = 0;

    public double maxAccelScale = 0.5;
    public double switchOverPoint = 0.5;


    //Orbital max torques = 1.23 Nm
    //Four motors = 4.94 Nm
    //Radius of wheels = 0.05 m
    //Force from wheels = 98.86 N
    //Mass of robot = 10.5 kg
    //Max Accel = 9.4 m/s^2
    public double maxAccel = 9.4; // m/s^2

    //Weight required to pull = 2.5 kg
    //Force required to pull = 24.5 N
    //Max Friction Accel = 2.33
    public double frictionAccel = 2; // m/s^2



    public void setTarget(double dis, double startVel){
        this.dis = dis;
        this.startVel = startVel;
        this.tA = -((2*startVel)-Math.sqrt(2*(2*Math.pow(startVel,2)+(3*maxAccel*maxAccelScale*dis)))/(maxAccel*maxAccelScale));
        calcABCs(dis, startVel, tA);
    }

    public void calcABCs(double d, double v, double t){
        this.a = -(6*d)/Math.pow(t,3) + (3*v)/(Math.pow(t,2));
        this.b = (6*d)/(Math.pow(t,2)) - (4*v)/t;
        this.c = v;
    }

    public double calcAccel(double curDis, double curVel, double curTime){
        if(curDis/dis > switchOverPoint){
            return considerFric(calcAccelA(curTime));
        }else{
            return considerFric(calcAccelB(curDis, curVel));
        }
    }

    public double calcAccelA(double curTime){
        return (2*a*curTime) + b;
    }

    public double calcAccelB(double curDis, double curVel){
        tB = (3*curDis)/curVel;
        calcABCs(curDis, curVel, tB);
        return b;
    }


    public double considerFric(double accel){
        return Math.signum(accel)*(Math.abs(accel)+frictionAccel);
    }

    public double getPow(double curDis, double curVel, double curTime){
        double accel = calcAccel(curDis, curVel, curTime);
        return (accel/maxAccel);
    }







}
