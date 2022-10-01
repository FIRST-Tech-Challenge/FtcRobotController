package org.firstinspires.ftc.teamcode.robots.GoneFishin;

import org.firstinspires.ftc.teamcode.util.Vector2;

public class Field {

    public FieldObject[] objects = new FieldObject[35];

    public Field(boolean isBlue){
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


    }

    //todo: gets closest pole of a certain height
    public Vector2 getClosestOfHeight(Vector2 pos, int h){
        int minIndex = 0;
        if(h == 1){
            for(int i = 10; i < 19; i++){

            }
        }
        return null;
    }

}


