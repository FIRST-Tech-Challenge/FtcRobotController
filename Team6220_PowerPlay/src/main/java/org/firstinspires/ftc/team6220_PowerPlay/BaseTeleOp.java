package org.firstinspires.ftc.team6220_PowerPlay;

abstract public class BaseTeleOp extends BaseOpMode{

    //Stick curve method. the function can be changed whenever
    public double stickCurve(double input){
        return (Math.signum(input)*(Math.pow(input, 2)));
    }

    public void teleOpDrive(float leftXpos, float leftYpos, float turnpos){
        //Saving the gamepad inputs into variables
        double LeftXPos = leftXpos;
        double LeftYPos = leftYpos;
        double TurnPos = turnpos;
        //bools for if the buttons are pressed (to use grabber)
        boolean GP2_xIsPressed = gamepad2.x;
        boolean GP2_aIsPressed = gamepad2.a;

        //Deadzones, 5 is the deadzone angle. Atan2 determines angle between the two sticks

        if(Math.abs(Math.atan2(LeftYPos, LeftXPos)) > 5){
            //Case for driving the robot left and right
            driveRobot(LeftXPos, 0, TurnPos);
        }else if(Math.abs(Math.atan2(LeftXPos, LeftYPos)) > 5){
            //Case for driving the robot up and
            driveRobot(0, LeftYPos, TurnPos);
        }else{
            //Case for if the deadzones limits are passed, the robot drives normally
            driveRobot(LeftXPos, LeftYPos, TurnPos);
        }

        //grabber open/close method attached to controller buttons
        if (GP2_xIsPressed) {  //press x to close
            openGrabber(false);
        } else if (GP2_aIsPressed){  //press a to open
            openGrabber(true);
        }
    }
};