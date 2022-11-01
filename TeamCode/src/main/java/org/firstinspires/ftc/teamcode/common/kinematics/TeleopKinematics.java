//package org.firstinspires.ftc.teamcode.common.Kinematics;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
//
//public class TeleopKinematics extends Kinematics{
//    ElapsedTime timer = new ElapsedTime();
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
//    private boolean firstMovement = true; //true when robot is stopped.  False while it is moving.
//    private boolean setClicksCycle = false;
//
//    public TeleopKinematics(GlobalPosSystem posSystem) {
//        super(posSystem); //runs Kinematics constructor
//    }
//
//    public void logic(){
//        if (!shouldSpline()){
//            if (firstMovement) type = Kinematics.DriveType.STOP; //later on, make it so that it only STOPS if it was previously moving
//            else if (shouldSnap()) type = Kinematics.DriveType.SNAP;
//            else type = Kinematics.DriveType.LINEAR;
//
////            setSplineReference(); //this is 1 loop off each time?
//        } else { //otherwise, spline
//            type = Kinematics.DriveType.SPLINE;
//        }
//
//        boolean shouldTableSpin = (rx != 0 || ry != 0);
//
//        switch(type){
//            case LINEAR:
//                setLinearSpin();
//                if (shouldTableSpin) setTableSpin();
//
//                // 7 / 8 items (missing translationPowerPercentage, but we don't set that here)
//                break;
//
//            case SNAP:
//                getRotTargetClicks(turnAmountW);
//                //rotate modules until target is hit
//                rightThrottle = 1;
//                leftThrottle = 1;
//                translateSwitchMotors = (int)wheelOptimization(lx, ly)[2];
//                rotationSwitchMotors = translateSwitchMotors;
//                translationPowerPercentage = 0.0;
//                rotationPowerPercentage = 1.0;
//                rotatePower = snapWheelPID.update(currentW);
//                spinPower = 0;
//
//                // 8/8 items
//                break;
//
//            case SPLINE:
//                setLinearSpin();
//                if (Math.abs(currentW - targetW) >= constants.degreeTOLERANCE){
//
//                    double throttle;
//                    if (lx != 0) throttle = 1;
//                    else throttle = (lx > ly ? Math.abs(ly/lx) : Math.abs(lx/ly));
//
//                    rightThrottle = (rotationSwitchMotors == 1 ? throttle : 1);
//                    leftThrottle = (rotationSwitchMotors == 1 ? 1 : throttle);
//                    translateSwitchMotors = 1; //robot is always going to spline forward I think
//                    rotationSwitchMotors = 1; //header optimization is not a thing
////                    if (Math.abs(splineReference - posSystem.getPositionArr()[3]) <= constants.degreeTOLERANCE){
////                        rotatePower = counteractSplinePID.update(posSystem.getPositionArr()[3]);
////                    }
//                    if (shouldTableSpin) setTableSpin();
//                } else{
//                    type = DriveType.LINEAR;
//                }
//                // 8/8 items
//                break;
//
//            case TURN:
//                //...
//                break;
//
//            case STOP: //this is NOT reset
//                rightThrottle = 0;
//                leftThrottle = 0;
//                spinPower = 0;
//                rotatePower = 0;
//                translationPowerPercentage = 0;
//                rotationPowerPercentage = 0;
//
//                // 6/8 items (missing switchMotors for rotation and translation, but we don't really need to change that)
//                break;
//
//            default:
//                type = DriveType.STOP;
//                break;
//        }
//
//        firstMovement = noMovementRequests();
//    }
//
//    public boolean shouldSnap(){
//        return (Math.abs(currentW - optimizedTargetW) >= constants.degreeTOLERANCE);
//    }
//
//    public boolean shouldSpline(){
//        double turnAmount = joystickL - prevJoystickL;
//        if(Math.abs(turnAmount) > 180){
//            turnAmount = 360 - Math.abs(turnAmount);
//        }
//        if (Math.abs(turnAmount) <= 45 && !firstMovement){
//            if (joystickL != prevJoystickL){ //if the joystick has actually moved
//                return true;
//            } else{ //the joystick has not moved
//                if (type==DriveType.SPLINE){
//                    return true;
//                } else{
//                    return false;
//                }
//                // return (type==DriveType.SPLINE); //if doing x and joystick has not moved, keep doing x.
//            }
//        } else return false;
//    }
//
//    public void setPos(){
//        //setting targets
//        trackJoystickL();
//        double[] wheelTargets = wheelOptimization(lx, ly);
//        double[] robotTargets = robotHeaderOptimization(rx, ry);
//
//        turnAmountW = wheelTargets[0]; //optimized turn amount
//        targetW = wheelTargets[1]; //target orientation for wheel
//        turnDirectionW = wheelTargets[2];
//
//        double robotTurnAmount = (lx == 0 && rx == 0 ? 0 : robotTargets[0]); //how much the robot header should turn
//
//        optimizedTargetW = clamp(currentW + turnAmountW); //optimized target orientation
//        targetR = clamp(currentR + robotTurnAmount); //the target orientation is on a circle of (-179, 180]
//
//        //setting PIDs for rotation of wheels & robot
//        snapWheelPID.setTargets(optimizedTargetW, 0.01, 0, 0);
//        tableSpinWheelPID.setTargets(targetR, 0.025, 0, 0);
//    }
//
//
//    public void setLinearSpin(){
//        spinPower = Math.sqrt(Math.pow(lx,2) + Math.pow(ly, 2));
//        rotatePower = 0;
//        if (Math.abs(spinPower) >= 1) spinPower = 1;
//        rightThrottle = 1;
//        leftThrottle = 1;
//        translationPowerPercentage = 1;
//        rotationPowerPercentage = 0;
//
//        getSpinTargetClicks();
//    }
//
//    public void setTableSpin(){
//        rotatePower = tableSpinWheelPID.update(currentR);
//        rotationSwitchMotors = 1; //optimization is not a thing for robot header, so we keep it always at 1.
//
//        double error = Math.abs(targetR - currentR);
//        if (error >= constants.degreeTOLERANCE){
//            rotationPowerPercentage = constants.tableSpinRotPercAllocation;
//            translationPowerPercentage = constants.tableSpinSpinPercAllocation;
//        } else{
//            translationPowerPercentage = 1.0;
//            rotationPowerPercentage = 1 - translationPowerPercentage;
//        }
////        setSplineReference();
//    }
//
//    public void getRotTargetClicks(double turnAmount){
//        if (setClicksCycle == false){
//            setClicksCycle = true;
//            rotClicks = (int)(turnAmount * constants.CLICKS_PER_DEGREE * turnDirectionW);
//        } else if (!shouldSnap()){
//            setClicksCycle = false;
//        }
//        spinClicks = 0;
//    }
//
//    public void getSpinTargetClicks(){
//        spinClicks = (int)(100 * spinPower * translationPowerPercentage);
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
//        motorPower[0] = spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //top left
//        motorPower[1] = spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //bottom left
//        motorPower[2] = spinPower * translationPowerPercentage * translateSwitchMotors * rightThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //top right
//        motorPower[3] = spinPower * translationPowerPercentage * translateSwitchMotors * rightThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //bottom right
//
//        return motorPower;
//    }
//
//    public int[] getClicks(){
//        int[] clicks = new int[4];
//        clicks[0] = spinClicks + rotClicks;
//        clicks[1] = -spinClicks + rotClicks;
//        clicks[2] = spinClicks + rotClicks;
//        clicks[3] = -spinClicks  + rotClicks;
//        return clicks;
//    }
//
//    // volatile boolean joystickStill;
//    // public void noMovementRequests(){
//    //     if (lx==0 && ly==0 && rx==0 && ry==0){
//    //         Thread doubleCheck = new Thread() {
//    //             public void run(){
//    //                 try {
//    //                     Thread.sleep(200);
//    //                     joystickStill = (lx==0 && ly==0 && rx==0 && ry==0);
//    //                 } catch(InterruptedException v) {
//    //                     System.out.println(v);
//    //                 }
//    //             }
//    //         };
//    //         doubleCheck.start();
//    //     } else{
//    //         joystickStill = false;
//    //     }
//    // }
//
//    public boolean noMovementRequests(){
//        return (lx==0 && ly==0 && rx==0 && ry==0);
//    }
//
//}
//
