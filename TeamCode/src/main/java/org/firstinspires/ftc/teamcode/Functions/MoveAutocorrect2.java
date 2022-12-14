package org.firstinspires.ftc.teamcode.Functions;

public class MoveAutocorrect2 {
    RotationDetector rotationDetector;
    Move move;
    Rotate rotate;
    double initialAngle;


    public MoveAutocorrect2(RotationDetector _RD, Move _MV, Rotate _RT){
        rotationDetector = _RD;
        move = _MV;
        rotate = _RT;
    }

    /**
     * This method saves and returns the initial angle.
     * @return : (double) This returns initial angle.
     */
    public double StartAngle(){
        initialAngle =rotationDetector.ReturnRotation();
        return initialAngle;
    }

    /**
     * This method changes the value of the initial angle.
     * @param angle : given angle
     */
    public void GivenAngle(double angle){
        initialAngle = angle;
    }

    /**
     * This method returns the diffrenece between the current and the initial angles.
     * @return : (double) This returns difference.
     */
    double Difference() {
        return (rotationDetector.ReturnRotation()- initialAngle);
    }

    /**
     * This method returns correction (variable) - corrects so that the robot stays on a straight line.
     * @return : (double) This returns correction.
     * The gain value determines how sensitive the correction is to direction changes.
     * You will have to experiment with your robot to get small smooth direction changes to stay on a straight line.
     */
    private double checkDirection()
    {
        double correction, difference, gain = .33;

        difference = Difference();

        if (difference == 0)
            correction = 0; // no adjustment.
        else
            correction = difference;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    int actualAngle;

    /**
     * This TWO methods are not used!
     * @return :
     */
    public int WaitForCorrection(){
        actualAngle = (int) rotationDetector.ReturnRotation();
        if(initialAngle > actualAngle) return 1;
        else if(initialAngle < actualAngle) return 2;
        else return 0;
    }

    public void CorrectAngle(){
        actualAngle = (int) rotationDetector.ReturnRotation();
        if(WaitForCorrection()==2) {
            while(rotationDetector.WaitForRotation((int) (actualAngle - Difference()))){
                rotate.RotateRaw(2, rotationDetector.MotorPower((int) (actualAngle - Difference())));
            }
            rotate.RotateStop();
        }
        else if(WaitForCorrection()==1){
            while(rotationDetector.WaitForRotation((int) (actualAngle - Difference()))){
                rotate.RotateRaw(1,rotationDetector.MotorPower((int) (actualAngle - Difference())));
            }
            rotate.RotateStop();
        }
    }

    /**
     * This method starts the robot and corrects throughout.
     * @param direction : 1 - goes front, 2 - goes back, 3 - goes left, 4 - goes right
     */
    public void MoveFull(int direction){

        if(Difference()==0){
            move.MoveFull(direction);
        }
        else{
            switch(direction){
                case 1: // Merge in fata
                    move.MoveOne(1, -1-checkDirection());
                    move.MoveOne(2, -1-checkDirection());
                    move.MoveOne(3, 1-checkDirection());
                    move.MoveOne(4, 1-checkDirection());
                    break;
                case 2:
                    // Merge in spate
                    move.MoveOne(1, 1-checkDirection());
                    move.MoveOne(2, 1-checkDirection());
                    move.MoveOne(3, -1-checkDirection());
                    move.MoveOne(4, -1-checkDirection());
                    break;
                case 3:
                    // lateral stanga
                    move.MoveOne(1, -1);
                    move.MoveOne(2, 1);
                    move.MoveOne(3, 1);
                    move.MoveOne(4, -1);
                    break;
                case 4:
                    // lateral dreapta
                    move.MoveOne(1, 1);
                    move.MoveOne(2, -1);
                    move.MoveOne(3, -1);
                    move.MoveOne(4, 1);
                    break;
            }

        }
}
}
