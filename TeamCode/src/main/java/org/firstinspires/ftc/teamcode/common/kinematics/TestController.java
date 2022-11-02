package org.firstinspires.ftc.teamcode.common.kinematics;

import android.app.backup.RestoreObserver;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;

public class TestController extends Kinematics {
    private double lx;
    private double ly;
    private double rx;
    private double ry;

//    private double joystickL = 0.0;
//    private double prevJoystickL = 0.0;
//    private int joystickCount = 0;

    public enum dType{
        LINEAR,
        SNAP,
        STOP,
        NOT_INITIALIZED
    }
    dType dtype = dType.NOT_INITIALIZED;

    Accelerator toplAccelerator = new Accelerator();
    Accelerator botlAccelerator = new Accelerator();
    Accelerator toprAccelerator = new Accelerator();
    Accelerator botrAccelerator = new Accelerator();

    public TestController(GlobalPosSystem posSystem) {
        super(posSystem); //runs Kinematics constructor
    }

    public void logic(){
        if (noMovementRequests()) dtype = dType.STOP;
        else if (shouldSnap()) dtype = dType.SNAP;
        else dtype = dType.LINEAR;
    }

    public boolean shouldSnap(){
        return (Math.abs(leftCurrentW - leftOptimizedTargetW) >= constants.degreeTOLERANCE || Math.abs(rightCurrentW - rightOptimizedTargetW) >= constants.degreeTOLERANCE);
    }

    public void setPos(){
        double[] leftWheelTargets = wheelOptimization(lx, ly, leftCurrentW);
        double[] rightWheelTargets =  wheelOptimization(lx, ly, rightCurrentW);
        double[] robotTargets = robotHeaderOptimization(rx, ry);

        leftTurnAmountW = leftWheelTargets[0]; //optimized turn amount
        rightTurnAmountW = rightWheelTargets[0];
        leftTargetW = leftWheelTargets[1]; //target orientation for wheel
        rightTargetW = rightWheelTargets[1];

        leftTurnDirectionW = (int)leftWheelTargets[2];
        rightTurnDirectionW = (int)rightWheelTargets[2];

        leftOptimizedTargetW = clamp(leftCurrentW + leftTurnAmountW); //optimized target orientation
        rightOptimizedTargetW = clamp(rightCurrentW + rightTurnAmountW);

        //setting PIDs for rotation of wheels & robot
        snapLeftWheelPID.setTargets(leftOptimizedTargetW, 0.03, 0, 0.01);
        snapRightWheelPID.setTargets(rightOptimizedTargetW, 0.03, 0, 0.01);
    }

    public double getRTargetW(){
        return rightTargetW;
    }
    public double getLTargetW(){
        return leftTargetW;
    }
    public int getRightDirectionW(){
        return rightTurnDirectionW;
    }
    public int getLeftDirectionW(){
        return leftTurnDirectionW;
    }
    public double getLTurnAmount(){
        return leftTurnAmountW;
    }
    public double getRTurnAmount(){
        return rightTurnAmountW;
    }
    public double getLOptimizedTargetW(){
        return leftOptimizedTargetW;
    }
    public double getROptimizedTargetW(){
        return rightOptimizedTargetW;
    }

//    private void trackJoystickL(){
//        joystickCount++;
//        if(joystickCount >= 3){
//            joystickCount = 0;
//            prevJoystickL = joystickL;
//            joystickL = Math.toDegrees(Math.atan2(lx, ly));
//        }
//    }

    public void getGamepad(double lx, double ly, double rx, double ry){
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;
    }

    public dType getdDriveType(){
        return dtype;
    }

    public boolean noMovementRequests(){
        return (lx==0 && ly==0 && rx==0 && ry==0);
    }
}

