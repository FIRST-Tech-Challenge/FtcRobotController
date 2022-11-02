package org.firstinspires.ftc.teamcode.common.kinematics;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.LinearCorrectionPID;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;

public class TurnKinematicsTest {
    protected Constants constants = new Constants();

    private double lx;
    private double ly;
    private double rx;
    private double ry;

    //robot's power
    double leftRotatePower = 0.0;
    double rightRotatePower = 0.0;
    double spinPower = 0.0;

    //target clicks
    public int rightRotClicks = 0;
    public int leftRotClicks = 0;
    public int spinClicks = 0; //make protected later

    public int spinDirectionR = 1;
    public int spinDirectionL = 1;

    double target;
    private enum Module{
        RIGHT,
        LEFT
    }

    //current orientation
    GlobalPosSystem posSystem;
    double leftCurrentW; //current wheel orientation
    double rightCurrentW;
    double currentR; //current robot header orientation


    Accelerator toplAccelerator = new Accelerator();
    Accelerator botlAccelerator = new Accelerator();
    Accelerator toprAccelerator = new Accelerator();
    Accelerator botrAccelerator = new Accelerator();


    public TurnKinematicsTest(GlobalPosSystem posSystem){
        this.posSystem = posSystem;

    }

    public void logic(){
        leftCurrentW = posSystem.getLeftWheelW();
        rightCurrentW = posSystem.getRightWheelW();
        currentR = posSystem.getPositionArr()[4];

        target = Math.toDegrees(Math.atan2(rx, ry));
        if (rx == 0 && ry == 0) target = 0;

        if (noMovementRequests()){
            spinPower = 0;
            leftRotatePower = 0;
            rightRotatePower = 0;

            spinClicks = 0;
            rightRotClicks = 0;
            leftRotClicks = 0;

//            translatePerc = 0;
//            rotatePerc = 0;
        } else if(shouldTurn()){
            robotHeaderOptimization(target);

            spinPower = Math.sqrt(Math.pow(rx,2) + Math.pow(ry, 2));;
            leftRotatePower = 0;
            rightRotatePower = 0;

            spinClicks = (int)(spinPower * 100); //currently, the robot won't stop turning once its hit its target.  WIll add that once we confirm this works.
            rightRotClicks = 0;
            leftRotClicks = 0;

        }
    }

    public void robotHeaderOptimization(double targetOrientation){
        //determine how much robot header must turn in which direction
        double turnAmount = targetOrientation-currentR;
        double turnDirection = Math.signum(turnAmount);
        if(Math.abs(turnAmount) > 180) {
            turnAmount = 360 - Math.abs(turnAmount);
            turnDirection *= -1;
        }
        if (turnDirection == -1){
            spinDirectionL *= -1;
            spinDirectionR *= 1;
        } else{
            spinDirectionR *= -1;
            spinDirectionL *= 1;
        }
    }

    public boolean shouldTurn(){
        return ((lx == 0 && ly == 0) && (rx != 0 || ry != 0));
    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;

        if (degrees < -179 || degrees > 180) {
            int modulo = (int)Math.signum(degrees) * -180;
            degrees = Math.floorMod((int)degrees, modulo);
        }
        return degrees;
    }

    public void getGamepad(double lx, double ly, double rx, double ry){
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;
    }


    public double[] getPower(){
        double[] motorPower = new double[4];

        motorPower[0] = spinPower + leftRotatePower; //top left
        motorPower[1] = spinPower + leftRotatePower; //bottom left
        motorPower[2] = spinPower + rightRotatePower; //top right
        motorPower[3] = spinPower + rightRotatePower; //bottom right

        toplAccelerator.update(motorPower[0]);
        botlAccelerator.update(motorPower[1]);
        toprAccelerator.update(motorPower[2]);
        botrAccelerator.update(motorPower[3]);

        return motorPower;
    }


    public int[] getClicks(){
        int[] clicks = new int[4];
        clicks[0] = spinClicks * spinDirectionL; //left
        clicks[1] = -spinClicks * spinDirectionL; //left
        clicks[2] = spinClicks * spinDirectionR; //right
        clicks[3] = -spinClicks * spinDirectionR; //right
        return clicks;
    }

    public boolean noMovementRequests(){
        return (lx==0 && ly==0 && rx==0 && ry==0);
    }

    public double getTarget(){
        return target;
    }
}

