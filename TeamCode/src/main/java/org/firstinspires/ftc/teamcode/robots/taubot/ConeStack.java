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
    public double z(){
        return 14 - 1.4*(5 - coneNum);
    }
//duck
}
