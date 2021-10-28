package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;

//class for operating claw
public class ServoClaw {

//var declarations
    private Servo claw;
    private float minturn;
    private float maxturn;

    //constructor accepts servo object, min and max turn params
    public ServoClaw(Servo claw, float minturn1, float maxturn1) {
        this.minturn = minturn1;
        this.maxturn = maxturn1;
        this.claw=claw;
    }
    //turns servo in range of min and max; percentage is used for pos
    public void actuateToPercent(float pos) {

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

    //returns percentage turned within range
    public double getPercentClosed(){
        double range = maxturn-minturn;
        return (claw.getPosition()-minturn)/range;

    }
    //returns current position
    public double getPos(){
        return claw.getPosition();
    }

    //intended to be looped; turns claw based on input speed from [-1,1]
    public void setClawSpeed(double speed) {

        //current position not child porn I swear
        double cp = claw.getPosition();

        //intended turn; /10 is just to slow it down a bit for control
        double tspeed = cp + (speed / 10);

        //overturn padding
        if (tspeed > maxturn) {
            claw.setPosition(maxturn);
        } else if (tspeed < minturn) {
            claw.setPosition(minturn);
        }
        //make intended turn if no padding
        else {
            claw.setPosition(tspeed);
        }
    }
}
