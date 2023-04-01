package org.firstinspires.ftc.teamcode.robots.taubot;

public class ConeStack extends FieldThing{

    int coneNum = 5;

    public ConeStack(String n, double y, double x, int h){
        super(n,y,x,h);
    }

    public void takeCone(){
        coneNum--;
    }

    @Override
    public double x(){
        return super.x()+0.05*(5 - coneNum);
    }

    @Override
    public double z(){
        return 11 - 1.5*(5 - coneNum);
    }

    public int getConeNum () {
        return coneNum;
    }
//duck
}
