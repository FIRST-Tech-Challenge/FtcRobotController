package org.firstinspires.ftc.teamcode.ultimategoal2020.fieldobjects2020;

import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ultimategoal2020.FieldPosition2020;

public class TowerGoal {
    //public double xPosition;
    //public double yPosition;
    public Alliance alliance;
    private FieldPosition2020 fieldPosition2020;

    public TowerGoal(){
        this.alliance = Alliance.BLUE;
        this.fieldPosition2020 = new FieldPosition2020(76, 36);
        this.fieldPosition2020.setyPosition(-this.fieldPosition2020.getyPosition());
    }

    public TowerGoal(Alliance a){
        this.fieldPosition2020 = new FieldPosition2020(76, 36);
       this.alliance = a;
       if (a == Alliance.RED){
           this.fieldPosition2020.setyPosition(-this.fieldPosition2020.getyPosition());
       }
    }

    public FieldPosition2020 getFieldPosition() {
        return fieldPosition2020;
    }
    public double getX(){
        return this.fieldPosition2020.getxPosition();
    }
    public double getY(){
        return this.fieldPosition2020.getyPosition();
    }
}
