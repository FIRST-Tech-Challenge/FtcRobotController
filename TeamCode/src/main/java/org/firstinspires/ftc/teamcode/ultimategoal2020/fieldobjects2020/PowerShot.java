package org.firstinspires.ftc.teamcode.ultimategoal2020.fieldobjects2020;

import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ultimategoal2020.FieldPosition2020;

public class PowerShot {

    public Alliance alliance;
    public TargetPosition targetPosition;
    private FieldPosition2020 fieldPosition2020;


    public enum TargetPosition{
        ONE, TWO, THREE
    }

    public PowerShot(TargetPosition tp, Alliance a) {
        this.alliance = a;
        this.targetPosition = tp;
        this.fieldPosition2020 = new FieldPosition2020(73, 0, 23.5);
        if(tp == TargetPosition.ONE){
            this.fieldPosition2020.setyPosition(3.5);
        }else if(tp == TargetPosition.TWO){
            this.fieldPosition2020.setyPosition(11.0);
        }else{
            this.fieldPosition2020.setyPosition(18.5);
        }

        if(a == Alliance.RED){
            this.fieldPosition2020.setyPosition(-this.fieldPosition2020.getyPosition());
        }

    }
    public FieldPosition2020 getFieldPosition(){
        return this.fieldPosition2020;
    }
}

