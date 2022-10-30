//package org.firstinspires.ftc.teamcode.common.Kinematics;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
//
//public class SplineKinematicsTest extends Kinematics{
//    ElapsedTime accelerationTimer = new ElapsedTime();
//    boolean isAccelerateCycle = false;
//
//    private double lx;
//    private double ly;
//    private double rx;
//    private double ry;
//
//    private double joystickL = 0.0;
//    private double prevJoystickL = 0.0;
//    private int joystickCount = 0;
//
//    public boolean inCycle = false;
//
//    public enum dType{
//        SPLINE,
//        STOP,
//    }
//    public dType dtype = dType.SPLINE;
//
//    public SplineKinematicsTest(GlobalPosSystem posSystem) {
//        super(posSystem); //runs Kinematics constructor
//        accelerationTimer.reset();
//    }
//
//    public void logic(){
//        inCycle = (Math.abs(joystickL - prevJoystickL) <= 5);
//        if (!inCycle || noMovementRequests()) dtype = dType.STOP;
//        else dtype = dType.SPLINE;
//
//        switch(dtype){
//            case SPLINE:
//                spinPower = Math.sqrt(Math.pow(lx,2) + Math.pow(ly, 2));
//                if ((rightCurrentW + leftCurrentW)/2.0 >= constants.degreeTOLERANCE){
//
//                    double throttle;
//                    double t = lx / 2.0;
//                    if (lx == 0) throttle = 1;
//                    else {
//                        throttle = (lx > ly ? Math.abs(ly/lx) : Math.abs(lx/ly));
//                        throttle = Math.abs((throttle - t)/(throttle + t));
//                    }
//
//                    int direction = (leftTurnDirectionW == rightTurnDirectionW ? leftTurnDirectionW : 1);
//                    rightThrottle = (direction == 1 ? throttle : 1);
//                    leftThrottle = (direction == 1 ? 1 : throttle);
////                    if (Math.abs(splineReference - posSystem.getPositionArr()[3]) <= constants.degreeTOLERANCE){
////                        rotatePower = counteractSplinePID.update(posSystem.getPositionArr()[3]);
////                    }
//                } else{
//                    leftThrottle = 1;
//                    rightThrottle = 1;
//                }
//                break;
//
//            case STOP: //this is NOT reset
//                rightThrottle = 1;
//                leftThrottle = 1;
//                spinPower = 0;
//                leftRotatePower = 0;
//                rightRotatePower = 0;
//                translationPowerPercentage = 0;
//                rotationPowerPercentage = 0;
//
//                rightRotClicks = 0;
//                leftRotClicks = 0;
//                spinClicks = 0;
//
//                isAccelerateCycle = false;
//
//                translateSwitchMotors = 1;
//
//                // 6/8 items (missing switchMotors for rotation and translation, but we don't really need to change that)
//                break;
//
//            default:
//                type = DriveType.STOP;
//                break;
//        }
//
//    }
//
//    public boolean shouldSnap(){
//        return (Math.abs(leftCurrentW - leftOptimizedTargetW) >= constants.degreeTOLERANCE || Math.abs(rightCurrentW - rightOptimizedTargetW) >= constants.degreeTOLERANCE);
//    }
//
//    public void setPos(){
//        //setting targets
//        trackJoystickL();
//
//        double[] robotTargets = robotHeaderOptimization(lx, ly);
//        targetR = robotTargets[1];
//        leftTurnAmountW =
//    }
//
//    public void setSplineClicks(){
//
//    }
//
//    private void trackJoystickL(){
//        joystickCount++;
//        if(joystickCount > 4){
//            joystickCount = 0;
//            prevJoystickL = joystickL;
//            joystickL = Math.toDegrees(Math.atan2(lx, ly));
//        }
//    }
//
//    public void getGamepad(double lx, double ly, double rx, double ry){
//        this.lx = lx;
//        this.ly = ly;
//        this.rx = rx;
//        this.ry = ry;
//    }
//
//    public double[] getPower(){
//        double[] motorPower = new double[4];
//
//        motorPower[0] = spinPower * translationPowerPercentage * leftThrottle + leftRotatePower * rotationPowerPercentage; //top left
//        motorPower[1] = spinPower * translationPowerPercentage * leftThrottle + leftRotatePower * rotationPowerPercentage; //bottom left
//        motorPower[2] = spinPower * translationPowerPercentage * rightThrottle + rightRotatePower * rotationPowerPercentage; //top right
//        motorPower[3] = spinPower * translationPowerPercentage * rightThrottle + rightRotatePower * rotationPowerPercentage; //bottom right
//
//        for (int i = 0; i < 4; i++){
//            motorPower[i] = accelerator(motorPower[i]);
//        }
//
//        return motorPower;
//    }
//
//    public double accelerator(double power){
//        if (power == 0) return 0.0;
//
//        if (!isAccelerateCycle){
//            accelerationTimer.reset();
//            isAccelerateCycle = true;
//        }
//        double accelerationFactor = (Math.tanh(3 * accelerationTimer.milliseconds() - 1.5) / 2.5) + 0.599;
//        power *= accelerationFactor;
//        return power;
//    }
//
//    public int[] getClicks(){
//        int[] clicks = new int[4];
//        clicks[0] = -splineSpinClicksL; //left
//        clicks[1] = splineSpinClicksL; //left
//        clicks[2] = -splineSpinClicksR; //right
//        clicks[3] = splineSpinClicksR; //right
//        return clicks;
//    }
//
//    public dType getdDriveType(){
//        return dtype;
//    }
//
//    public boolean noMovementRequests(){
//        return (lx==0 && ly==0 && rx==0 && ry==0);
//    }
//
//}
//
