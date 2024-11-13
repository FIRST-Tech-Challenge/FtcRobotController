package org.firstinspires.ftc.teamcode;
/*
To get the Trim to work
1) multiply the left motor by the value of the left Trim using the method trimLeft() wherever you want to Trim it
2) multiply the right motor by the value of the right Trim using the method trimRight() wherever you want to Trim it
3) call the update() method before you first move your motors in your while (opModeIsActive) loop
*/

class PowerLevels{
    private float leftPower = 1f;
    private float rightPower = 1f;

    public PowerLevels(float leftPower, float rightPower){
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }

    public float getLeftPower(){
        return leftPower;
    }

    public float getRightPower(){
        return rightPower;
    }
}
public class Trim {
    private float leftTrim = 1f;
    private float rightTrim = 1f;
    private final float TRIMAMOUNT = 0.01f;

    public float getLeftTrim(){
        return leftTrim;
    }

    public float getRightTrim(){
        return rightTrim;
    }

    public void addLeft(){

        if(rightTrim == 1)
        {
            leftTrim = Math.max(0,leftTrim-TRIMAMOUNT);
        }
        else
        {
            rightTrim = Math.min(rightTrim + TRIMAMOUNT, 1);
        }
    }
    public void addRight(){

        if(leftTrim == 1)
        {
            rightTrim = Math.max(0,rightTrim-TRIMAMOUNT);
        }
        else
        {
            leftTrim = Math.min(leftTrim + TRIMAMOUNT, 1);
        }
    }

    public PowerLevels getPowerLevel(float left, float right){
        return new PowerLevels(left*leftTrim,right*leftTrim);
    }

}
