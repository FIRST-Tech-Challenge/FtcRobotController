package org.firstinspires.ftc.teamcode.robots.gruntbuggly;

import org.firstinspires.ftc.teamcode.util.Vector2;

public class Field {

    public FieldObject[] objects = new FieldObject[35];
    public boolean isBlue;

    public Field(boolean isBlue){
        this.isBlue = isBlue;
        if(isBlue){
            objects[0] = new FieldObject("AllianceTerminalClose",2.5,0.5,0);
            objects[1] = new FieldObject("AllianceTerminalFar",-2.5,5.5,0);
            objects[2] = new FieldObject("EnemyTerminalClose",-2.5,0.5,0);
            objects[3] = new FieldObject("EnemyTerminalFar",2.5,5.5,0);
        }else{
            objects[0] = new FieldObject("AllianceTerminalClose",-2.5,0.5,0);
            objects[1] = new FieldObject("AllianceTerminalFar",2.5,5.5,0);
            objects[2] = new FieldObject("EnemyTerminalClose",2.5,0.5,0);
            objects[3] = new FieldObject("EnemyTerminalFar",-2.5,5.5,0);
        }

        objects[4] = new FieldObject("AllianceLeftStack",-2.5,2.5,0);
        objects[5] = new FieldObject("AllianceRightStack",2.5,2.5,0);
        objects[6] = new FieldObject("EnemyLeftStack",-2.5,3.5,0);
        objects[7] = new FieldObject("EnemyRightStack",2.5,3.5,0);

        objects[8] = new FieldObject("AllianceSubstation",0,0.5,0);
        objects[9] = new FieldObject("EnemySubstation",0,5.5,0);

        for(int i = 0; i < 3 ; i++){
            for(int j = 0; j < 3; j++){
                objects[i*3+j+10] = new FieldObject("GroundStation"+(i*3+j+1),-2 + j*2,5-i*2,1);
            }
        }

        objects[19] = new FieldObject("LowPole1",-1,5,2);
        objects[20] = new FieldObject("LowPole2",1,5,2);
        objects[21] = new FieldObject("LowPole3",-2,4,2);
        objects[22] = new FieldObject("LowPole4",2,4,2);
        objects[23] = new FieldObject("LowPole5",-2,2,2);
        objects[24] = new FieldObject("LowPole6",2,2,2);
        objects[25] = new FieldObject("LowPole7",-1,1,2);
        objects[26] = new FieldObject("LowPole8",1,1,2);


        objects[27] = new FieldObject("MidPole1",-1,4,3);
        objects[28] = new FieldObject("MidPole2",1,4,3);
        objects[20] = new FieldObject("MidPole3",-1,2,3);
        objects[30] = new FieldObject("MidPole4",1,2,3);

        objects[31] = new FieldObject("HighPole1",0,4,4);
        objects[32] = new FieldObject("HighPole2",-1,3,4);
        objects[33] = new FieldObject("HighPole3",1,3,4);
        objects[34] = new FieldObject("HighPole4",0,2,4);

        objects[35] = new FieldObject("AllianceSignal1",-1.5,1.5,-1);
        objects[36] = new FieldObject("AllianceSignal2",1.5,1.5,-1);

    }

    public void changeOwnership(int ID, boolean isBlue, boolean isRed){
        objects[ID].setOwnership(isBlue,isRed);
    }

    //todo: gets closest pole of a certain height
    public int GetNearest(int h,Vector2 pos){
        int minIndex = 0;
        double min = 999;
        switch(h){
            case 1:
                for(int i = 10; i < 19; i++){
                    if(Vector2.magnitude(objects[i].getPosition().subtract(pos)) < min){
                        minIndex = i;
                    }
                }
                break;

            case 2:
                for(int i = 19; i <27; i++){
                    if(Vector2.magnitude(objects[i].getPosition().subtract(pos)) < min){
                        minIndex = i;
                    }
                }
                break;

            case 3:
                for(int i = 27; i < 31; i++){
                    if(Vector2.magnitude(objects[i].getPosition().subtract(pos)) < min){
                        minIndex = i;
                    }
                }
                break;

            case 4:
                for(int i = 31; i < 35; i++){
                    if(Vector2.magnitude(objects[i].getPosition().subtract(pos)) < min){
                        minIndex = i;
                    }
                }

            case -1:
                for(int i = 35; i < 37; i++){
                    if(Vector2.magnitude(objects[i].getPosition().subtract(pos)) < min){
                        minIndex = i;
                    }
                }

            default:
                break;

        }
        return minIndex;
    }

}


