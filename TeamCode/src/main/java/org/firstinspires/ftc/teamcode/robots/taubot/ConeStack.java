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
        return super.x()+0.03*(5 - coneNum);
    }

    @Override
    public double z(){
        return 12 - 1.4*(5 - coneNum);
    }
//duck
}
