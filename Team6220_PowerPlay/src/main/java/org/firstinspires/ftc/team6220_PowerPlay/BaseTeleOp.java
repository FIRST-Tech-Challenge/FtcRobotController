package org.firstinspires.ftc.team6220_PowerPlay;

abstract public class BaseTeleOp extends BaseOpMode{

    //Stick curve method. the function can be changed whenever
    public double stickCurve(double input){
        return (Math.signum(input)*(Math.pow(input, 2)));
    }

    // servo testing stuff
    double servoPos = 0.0;

    public void teleOpDrive(){
        //Saving the gamepad inputs into variables
        double LeftXPos = stickCurve(gamepad1.left_stick_x);
        double LeftYPos = stickCurve(gamepad1.left_stick_y);
        double TurnPos = stickCurve(gamepad1.right_stick_x);

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

        // servo testing stuff
        if (gamepad2.dpad_up) {
            servoPos += 0.5;
        } else if (gamepad2.dpad_down) {
            servoPos -= 0.5;
        }
        servoGrabber.setPosition(servoPos);
        telemetry.addData("servoPos", servoPos);
        telemetry.update();
    }
};