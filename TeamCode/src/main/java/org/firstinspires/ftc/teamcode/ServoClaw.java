package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;

//class for operating claw
public class ServoClaw {

//var declarations
    private Servo claw;
    private float minturn;
    private float maxturn;

    //constructor accepts min and max turn params
    public ServoClaw(Servo claw, float minturn1, float maxturn1) {
        this.minturn = minturn1;
        this.maxturn = maxturn1;
    }
    //turns servo in range of min and max; percentage is used for pos
    public void turninrange(float pos) {

        //corrects out of domain pos values
        if(pos>1){
            pos=1;
        }
        else if(pos<0){
            pos=0;
        }

        //calculates percentage and turns based on it
        float range = maxturn-minturn;
        float finpos = (pos*range)+minturn;
        claw.setPosition(finpos);
    }

}
