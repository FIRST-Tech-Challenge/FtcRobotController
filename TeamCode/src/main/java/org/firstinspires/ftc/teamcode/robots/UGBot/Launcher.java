package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.SharpDistanceSensor;

import static org.firstinspires.ftc.teamcode.util.Conversions.futureTime;
import static org.firstinspires.ftc.teamcode.util.Conversions.servoNormalize;
import static org.firstinspires.ftc.teamcode.vision.Config.SERVO_MAX;

/**
 * Created by 2938061 on 11/10/2017.
 */
@Config
public class Launcher {

    //number of ticks per revolution REV HD motor: 2240
    //number of ticks per revolution REV Core    : 288

    DcMotor elbow = null;
    DcMotor extendABob = null;
    Servo hook = null;

    PIDController extendPID;
    public static double kpExtendABob = 0.006; //proportional constant multiplier goodish
    public static  double kiExtendABob = 0.0; //integral constant multiplier
    public static  double kdExtendABob= 0.0; //derivative constant multiplier
    double extendCorrection = 0.00; //correction to apply to extention motor

    PIDController elbowPID;
    public static double kpElbow = 0.006; //proportional constant multiplier goodish
    public static  double kiElbow = 0.0; //integral constant multiplier
    public static  double kdElbow= 0.0; //derivative constant multiplier
    double elbowCorrection = 0.00; //correction to apply to elbow motor
    boolean elbowActivePID = true;
    boolean extendABobActivePID = true;

    Servo intakeRight = null;
    Servo intakeLeft = null;
    Servo servoGripper = null;
    Servo intakeServoBack = null;
    Servo gripperSwivel = null;

    SharpDistanceSensor gripLeftSharp;
    SharpDistanceSensor gripRightSharp;

    public static double gripLeftDist;
    public static double gripRightDist; //these hold the most recently updated values for the gripper distance sensors



    int elbowPosInternal = 0;
    int elbowPos = 0;
    double elbowPwr = 0;

    int extendABobPosInternal = 0;
    int extendABobPos = 0;
    double extendABobPwr = 0;

    int intakeState = 3;
    boolean beltToElbowEnabled;

    public int motorHooked;
    public int motorUnhooked;
    public int motorMidHooked;

    int servoGripperOpen;
    int servoGripperClosed;

    public double intakePwr;
    //normal Teleop encoder values
    public int pos_preIntake;
    public int pos_Intake ;
    public int pos_Deposit;
    public int pos_reverseIntake;
    public int pos_reversePreDeposit;
    public int pos_reverseDeposit;
    public int pos_reverseSafeDrive;
    public int pos_PartialDeposit;
    public int pos_SafeDrive;
    public  int swivel_Right90;
    public int swivel_Calibrate;
    public int swivel_Front;
    public  int swivel_Left90;
    public int swivel_left_Block;
    public int swivel_Right_Block;
    private boolean gripperState;
    private int gripperSwivelState = 0;
    double hypotenuse = 0;

    //autonomous encoder values
    public int pos_AutoPark;
    public int pos_autonPrelatch;

    //end game encoder values
    public int pos_prelatch;
    public int pos_latched;
    public int pos_postlatch;

    //.374

    //elbow safety limits
    public int elbowMin = -50;
    public int elbowStart = 180; //put arm just under 18" from ground
    public int elbowLow = 300;
    public int elbowMinCalibration = -1340; //measure this by turning on the robot with the elbow fully opened and then physically push it down to the fully closed position and read the encoder value, dropping the minus sign
    public int actualElbowMax = 1120;
    public int elbowMid = (actualElbowMax + elbowMin)/2;
    public int elbowMaxSafetyOffset = 70; //makes sure that the robot doesn't try and extend to the elbow max exactly

    //belt extension encoder values
    public  int extendDeposit;
    public  int extendMax;
    public  int extendMid;
    public  int extendLow; //clears hook and good for retracting prior to deposit without tipping robot
    public  int extendMin;  //prevent crunching collector tray
    public  int elbowBrigeTransit;
    public  int extensionBridgeTransit;

    //foundation hook servo values
    public int foundation_hook_up = 1665;
    public int foundation_hook_down = 1163;

    public int currentTowerHeight;
    public static final double blockHeightMeter = 0.127;

    private boolean hookUp = true;
    //private int gripperState = 0;

    public double ticksPerDegree = 19.4705882353;
    public final double ticksPerMeter = 806/.2921;
    public static final double DISTANCE_SENSOR_TO_ELBOW = 0.33;
    public static final double GRIPPER_HEIGHT = 0.23;
    public static final double Y_LEEWAY = 0.05;
    public static final double X_LEEWAY = 0.02;
    public static final double ELBOW_HEIGHT = 0.24;
    public static final double CRANE_LENGTH = .3683;
    public boolean active = true;

    public boolean getGripperState() {
        return gripperState;
    }





    public Launcher(DcMotor elbow, DcMotor extendABob, Servo hook, Servo servoGripper, Servo intakeServoBack, Servo gripperSwivel){

        //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setTargetPosition(elbow.getCurrentPosition());
//        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //elbow.setDirection(DcMotorSimple.Direction.REVERSE);

        //extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendABob.setTargetPosition(extendABob.getCurrentPosition());
//        extendABob.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABob.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //hook.setDirection(DcMotorSimple.Direction.REVERSE);

        this.elbow = elbow;
        this.extendABob = extendABob;
        this.hook = hook;
        this.servoGripper = servoGripper;
        this.intakeServoBack = intakeServoBack;
        this.gripperSwivel = gripperSwivel;
        intakeServoBack.setDirection(Servo.Direction.REVERSE);
        intakePwr = .3; //.35;
        //normal Teleop encoder values
        pos_preIntake = 3600;
        pos_Intake   = 2660;
        pos_Deposit  = 1520;
        pos_reverseIntake = 1407;
        pos_reversePreDeposit=1408;
        pos_reverseDeposit = 3400;
        pos_reverseSafeDrive = 1000;
        pos_PartialDeposit = 1700;
        pos_SafeDrive = 800;
        currentTowerHeight = 0;

        //autonomous encoder values
        pos_AutoPark = pos_SafeDrive + 500;
        pos_autonPrelatch = 2950;

        //end game encoder values
        pos_prelatch = 2000;
        pos_latched = 2764;
        pos_postlatch = 1240;

        servoGripperOpen = 1350;
        servoGripperClosed = 800;

        motorHooked = 120;
        motorUnhooked = 5;
        motorMidHooked = 80;

        swivel_Calibrate = 1200;
        swivel_Right90 = 900;
        swivel_Front = 1600;
        swivel_Left90 = 2100;
        swivel_left_Block = 800;
        swivel_Right_Block= 1000;

        //belt extension encoder values
        extendDeposit = 1489;
        extendMax = 2700;
        extendMid= 980;
        extendLow = 600; //clears foundation grabber at all times
        extendMin = 300;  //prevent crunching foundation grabber
        elbowBrigeTransit = 230;
        extensionBridgeTransit = 500;
        gripperState = false;

        //PID
        extendPID = new PIDController(0,0,0);
        elbowPID = new PIDController(0,0,0);
        elbowPID.setIntegralCutIn(40);
        elbowPID.enableIntegralZeroCrossingReset(false);


    }


    public void update(){
        gripLeftDist = gripLeftSharp.getUnscaledDistance(); //remove these two lines if looking for raw voltage which goes up with proximity
        gripRightDist = gripRightSharp.getUnscaledDistance();

        updateGripper();
        updateBeltToElbow();

        //

        if(active) {// && elbowPosInternal!=elbowPos) { //don't keep updating if we are retractBelt to target position
            //elbowPosInternal = elbowPos;
//            elbow.setTargetPosition(elbowPos);
            if(elbowActivePID)
                movePIDElbow(kpElbow, kiElbow, kdElbow, elbow.getCurrentPosition(), elbowPos);
            else
                elbowPos = elbow.getCurrentPosition();

        }
        if(active) {// && extendABobPosInternal!=extendABobPos) { //don't keep updating if we are retractBelt to target position
            //extendABobPosInternal = extendABobPos;
//            extendABob.setTargetPosition(extendABobPos);
            if(extendABobActivePID)
                movePIDExtend(kpExtendABob, kiExtendABob, kdExtendABob, extendABob.getCurrentPosition(), extendABobPos);
            else
                extendABobPos = extendABob.getCurrentPosition();
        }
    }

    public void movePIDExtend(double Kp, double Ki, double Kd, double currentTicks, double targetTicks) {

        //initialization of the PID calculator's output range, target value and multipliers
        extendPID.setOutputRange(-extendABobPwr, extendABobPwr);
        extendPID.setPID(Kp, Ki, Kd);
        extendPID.setSetpoint(targetTicks);
        extendPID.enable();

        //initialization of the PID calculator's input range and current value
        //extendPID.setInputRange(0, 360);
        //extendPID.setContinuous();
        extendPID.setInput(currentTicks);

        //calculates the correction to apply
        extendCorrection = extendPID.performPID();

        //performs the extension with the correction applied
        extendABob.setPower(extendCorrection);
    }

    public void movePIDElbow(double Kp, double Ki, double Kd, double currentTicks, double targetTicks) {

        //initialization of the PID calculator's output range, target value and multipliers
        elbowPID.setOutputRange(-elbowPwr, elbowPwr);
        elbowPID.setPID(Kp, Ki, Kd);
        elbowPID.setSetpoint(targetTicks);
        elbowPID.enable();

        //initialization of the PID calculator's input range and current value
        elbowPID.setInput(currentTicks);

        //calculates the correction to apply
        elbowCorrection = elbowPID.performPID();

        //moves elbow with the correction applied
        elbow.setPower(elbowCorrection);
    }

    public void updateBeltToElbow() {
        if(beltToElbowEnabled) {
            setElbowTargetPos(beltToElbow(getExtendABobCurrentPos(), 0)-40);
        }
    }

    public void setBeltToElbowModeEnabled() {
        beltToElbowEnabled = true;
    }

    public void setBeltToElbowModeDisabled() {
        beltToElbowEnabled = false;
    }

    public int elbowToBelt(int elbow, int offset){
        return (int)(4.5*(elbow+ offset)) +620;
    }

    public int beltToElbow(int belt, int offset){
        return (int)(2.0/9 * ((belt+offset)-620)) ;
    }

    int grabState = 2;
    double grabTimer;

    public void updateGripper() {
        switch(grabState){
            case 0:
                servoGripper.setPosition(servoNormalize(SERVO_MAX));
                //if(setElbowTargetPos(elbow.getCurrentPosition(),.2)) {
                    grabTimer = futureTime(1);
                    grabState++;
                //}
                break;

            case 1:
                if (System.nanoTime() >= grabTimer) {
                    servoGripper.setPosition(servoNormalize(1700));
                    grabState++;
                }
                break;

        }

    }

    public void changeTowerHeight(int newHeightAddition){
        if(currentTowerHeight > 0 || newHeightAddition > 0)
        currentTowerHeight += newHeightAddition;
    }

    public int getCurrentTowerHeight(){
        return currentTowerHeight;
    }

    public int getElbowMinCalibration() {return elbowMinCalibration;}

    public void extendToTowerHeight(double distance, int stackHeight) {
        double x = distance + DISTANCE_SENSOR_TO_ELBOW - X_LEEWAY;
        double y = (stackHeight + 1) * blockHeightMeter + GRIPPER_HEIGHT + Y_LEEWAY - ELBOW_HEIGHT;
        setElbowTargetAngle(Math.toDegrees(Math.atan(y / x))); //in degrees
        setExtendABobLengthMeters(Math.sqrt(x * x + y * y) - CRANE_LENGTH); //in meters
    }

    public void extendToTowerHeight(){
        hypotenuse = Math.sqrt(.76790169 + Math.pow(((currentTowerHeight+1)* blockHeightMeter),2));//in meters
        setElbowTargetAngle(Math.toDegrees(Math.acos(0.8763/ hypotenuse)));
        setExtendABobLengthMeters(hypotenuse-.3683);
    }

    public void extendToTowerHeight(int height){
        hypotenuse = Math.sqrt(.76790169 + Math.pow(((height+1)* blockHeightMeter),2));//in meters
        setElbowTargetAngle(Math.toDegrees(Math.acos(0.8763/ hypotenuse)));
        setExtendABobLengthMeters(hypotenuse-.3683);
    }

    public void hookOn(){

        hook.setPosition(servoNormalize(foundation_hook_down));
        hookUp = false;
    }
    public void hookOff(){
        hook.setPosition(servoNormalize(foundation_hook_up));
        hookUp = true;
    }

    public void hookToggle(){
        if(hookUp)
            hookOn();
        else
            hookOff();
    }

    public void swivelGripper(boolean right){
        if(right == true)
            gripperSwivel.setPosition(gripperSwivel.getPosition()-.02);
        else
            gripperSwivel.setPosition(gripperSwivel.getPosition()+.02);
    }

    public void swivelGripperSlow(boolean right){
        if(right == true)
            gripperSwivel.setPosition(gripperSwivel.getPosition()-.01);
        else
            gripperSwivel.setPosition(gripperSwivel.getPosition()+.01);
    }

    public boolean alignGripperForwardFacing() {

        //todo: test and fix this
        //continuously try to align the gripper with a stone
        //this version assumes forward or sideways (parallel to ground) directed sharp distance sensors that can get a distance to the stone
        //A hint of orientation will help - though which sensor triggers first might tell us something
        //once either sensor detects the stone, we can start rotating in favor of equalizing distances
        //when distances are roughly equal and below the trigger threshold - that's when we yoink
        //experiment with waiting on rotation until both sensors see something. this would reduce the chances of aligning one sensor to the broad side and the other to the narrow side


        double stoneDistMax = .8; //beyond this distance we should assume we are not trying to do anything
        double stoneTriggerDist = .22; //what is the typical trigger distance to yoink the stone - previously .13

        if (gripLeftDist > stoneDistMax || gripRightDist > stoneDistMax)
            return false; // we are too far away on one sensor or the other

        double diff = gripRightDist - gripLeftDist;
        if (Math.abs(diff) > .065){ //test to see if the difference is even slightly significant - hysteresis
            if (diff < 0.0)
                swivelGripperSlow(true);
            else
                swivelGripperSlow(false);
        }
        if (gripLeftDist < stoneTriggerDist && gripRightDist < stoneTriggerDist) return true;

        return false;

    }

    public boolean alignGripperDownFacing(){
//todo: this likely broke when switching to linearized sharp distance sensor interpretation - fix
        //continuously try to align the gripper with a stone
        //this version assumes downward directed sharp distance sensors that only see the stone when they cross over its edge
        //should not operate while both sensors are seeing a larger distance to the floor than is reasonable
        //when both sensors see the stone (distance indicates stone height above floor) then call it aligned, return true;
        //if only one sensor sees the stone, rotate so that sensor backs away
        //if both sensors only see the floor, extend crane out a bit
        //repeat
        double stoneDistMin = 1.0; //what is the typical trigger distance to a stone?
        double stoneDistMax = 1.4;
        double floorDist = .4; //what is the typical trigger distance to the floor when at approach height?

        if (gripLeftDist < floorDist || gripRightDist < floorDist) return false; // we are too high

        //if (gripLeftDist < stoneDistMax && gripRightDist < stoneDistMax) return true; //we think we are done - could easily be over extended if we didn't approach correctly

        if (gripLeftDist < stoneDistMax && gripLeftDist > stoneDistMin) {// && gripLeftDist < stoneDist && gripRightDist > floorDist) {
            //we might be seeing a stone with the left sensor, so swivel left
            swivelGripper(false);
        }
            if (gripRightDist < stoneDistMax && gripRightDist > stoneDistMin){// && gripLeftDist < stoneDist && gripRightDist > floorDist) {
                //we might be seeing a stone with the left sensor, so swivel left
                swivelGripper(true);

            }


        return false;

    }

    public void toggleSwivel(){
        if(gripperSwivelState == 0) {
            gripperSwivel.setPosition(.5);
            gripperSwivelState++;
        }
        else if(gripperSwivelState == 1) {
            gripperSwivel.setPosition(1);
            gripperSwivelState++;
        }
        else if(gripperSwivelState == 2) {
            gripperSwivel.setPosition(.5);
            gripperSwivelState++;
        }
        else{
            gripperSwivel.setPosition(0);
            gripperSwivelState = 0;
        }
    }

    //This is for auto
    public boolean setGripperSwivelRotation(int encodedPosition){
        gripperSwivel.setPosition(servoNormalize(encodedPosition));
        return true;
    }

    public boolean grabStone(){
        servoGripper.setPosition(servoNormalize(servoGripperClosed));

        //gripperState = 1;
        return true;
    }
    public boolean ejectStone(){
        servoGripper.setPosition(servoNormalize(servoGripperOpen));
        //gripperState = 2;
        return true;
    }
    public boolean setGripperPos(boolean open){
        if(open)
            servoGripper.setPosition(servoNormalize(servoGripperOpen));
        else
            servoGripper.setPosition(servoNormalize(servoGripperClosed));
        return true;
    }
    public void stopGripper() {
        servoGripper.setPosition(servoNormalize(1500));

        //gripperState = 0;
    }

    public void stopIntake(){

    }

    //this is a behavior to calibrate the crane including the elbow, arm and gripper
    //keep calling until it returns true
    //arm should be pointed mostly up and mostly retracted
    //first raise the elbow on low power so it stalls on physical limit of hitting the extension motor shaft
    //must be run without encoders so power won't ramp up
    //retract the arm (extendabob) until it stops on low power - this will be its zero position

    int calibrateStage=0;
    double calibrateTimer;

    public boolean calibrate(){

        switch(calibrateStage){
            case 0: //open elbow with limited power until it stalls at top of travel
                // retract extendabob same way
                setElbowActivePID(false);
                setExtendABobPwr(1);
                setElbowPwr(1);
                elbow.setPower(.20);
                setExtendABobActivePID(false);
                extendABob.setPower(-.25); //retract to zero position
                setGripperSwivelRotation(swivel_Calibrate); //put gripper in position where it doesn't tangle belts
                calibrateTimer = futureTime(3.0f); //allow enough time for elbow to open fully and arm to retract
                calibrateStage++;
                break;
            case 1: //reset encoders at max elevation angle
                if (System.nanoTime() >= calibrateTimer){
                    extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // this approximately zeros the extension but at elbowmax and we'll need to redo it at a low elevation
                    extendABob.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setExtendABobActivePID(true);
                    extendToLow(); //push out to the minimum extension - this will need to be pulled back to zero for starting position


                    elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //temporarily zero at top of travel
                    elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //elbow.setTargetPosition(-elbowMinCalibration); //normally we set the target through a method, but we have to override the safety here
                    setElbowMin(elbowMinCalibration);
                    setElbowActivePID(true);
                    setElbowTargetPos(elbowMinCalibration);


                    //elbow.setPower(1); //power down to low position
                    calibrateTimer = futureTime(1.5f); //allow enough time for elbow to close fully
                    calibrateStage++;

                }
                break;
            case 2: //zero the elbow at bottom of travel
                if (System.nanoTime() >= calibrateTimer) {
                    elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero elbow at bottom of travel
                    elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setElbowMin(-50);
                    //elbow.setTargetPosition(elbowMin); //this should not generate a movement because we should already be there
                    setElbowActivePID(true);
                    setElbowTargetPos(elbowMin);
                    calibrateTimer = futureTime(1f); //allow enough time to raise to starting position
                    calibrateStage++;
                }
                break;
            case 3: //reset elbow to near starting position so we can get a better retract for extension
                if (System.nanoTime() >= calibrateTimer) {
                    //elbow.setTargetPosition(elbowLow);
                    setElbowTargetPos(elbowLow);
                    calibrateTimer = futureTime(2.0f); //enough time for next stage - retract egain
                    calibrateStage++;

                }
                break;
            case 4: //one more retract of extension
                if (System.nanoTime() >= calibrateTimer) {
                    setExtendABobActivePID(false);
                    extendABob.setPower(-.6); //retract to zero position
                    calibrateTimer = futureTime(1.0f); //allow enough time for next stage
                    calibrateStage++;
                }
                break;

            case 5: //final zero of extension
                if (System.nanoTime() >= calibrateTimer) {
                    extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // this should be correct zero for extension
                    extendABob.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setExtendABobActivePID(true);
                    extendToPosition(0,.6);
                    //elbow.setTargetPosition(elbowStart);
                    setElbowTargetPos(elbowStart);
                    calibrateStage = 0;
                    return true;
                }
                break;


        }


        return false;
    }

    public void setMotorsForCalibration(){

            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendABob.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setElbowActivePID(boolean isActive){elbowActivePID = isActive;}
    public void setExtendABobActivePID(boolean isActive){extendABobActivePID = isActive;}


    public boolean toggleGripper() {
        grabState = 0;
        return true;
    }


    public void collect(){
        intakeLeft.setPosition(.5 + intakePwr);
        intakeRight.setPosition(.5 + intakePwr);
    }
//    public void eject(){
//        intakeRight.setPosition(.5 - intakePwr);
//        intakeLeft.setPosition(.5 - intakePwr);}
//    public void stopIntake(){
//        intakeRight.setPosition(.5);
//        intakeLeft.setPosition(.5);
//    }


    private void setExtendABobTargetPos(int pos){
        extendABobPos = Math.min(Math.max(pos, extendMin),extendMax);
    }
    private void setExtendABobTargetPosUnsafe(int pos){
        extendABobPos = Math.min(Math.max(pos, 0),extendMax);
    }

    public void setExtendABobTargetPosUnsafeReally(int pos){
        extendABobPos = pos;
    }

    public int getExtendABobTargetPos(){
        return extendABobPos;
    }
    public int getExtendABobCurrentPos(){
        return extendABob.getCurrentPosition();
    }
    public void setExtendABobPwr(double pwr){ extendABobPwr = pwr; }


    private void setElbowTargetPos(int pos){
        elbowPos = Math.min(Math.max(pos, elbowMin), actualElbowMax -elbowMaxSafetyOffset);
    }

    public void setElbowTargetPosNoCap(int pos){
            elbowPos = pos;
    }

    private void setElbowMin(int newMin){elbowMin = newMin;}

    public boolean setElbowTargetPos(int pos, double speed){
        setElbowTargetPos(pos);
        setElbowPwr(speed);
        if (nearTargetElbow()) return true;
        else return false;
    }


    public boolean setElbowTargetAngle(double angleDegrees){
        setElbowTargetPos((int) (ticksPerDegree* angleDegrees));
        return true;
    }
    public int getElbowTargetPos(){
        return elbowPos;
    }
    public int getElbowCurrentPos(){
        return elbow.getCurrentPosition();
    }
    public double getCurrentAngle(){return  elbow.getCurrentPosition()/ticksPerDegree;}

    public void setExtendABobLengthMeters(double lengthMeters){
        setExtendABobTargetPos((int)(lengthMeters*ticksPerMeter));
    }

    public double getCurrentLengthInMeters(){
        return (ticksPerMeter)*getExtendABobCurrentPos();
    }
    public void setElbowPwr(double pwr){ elbowPwr = pwr; }

    public void stopAll(){
        setElbowPwr(0);
        setExtendABobPwr(0);
        setElbowActivePID(false);
        setExtendABobActivePID(false);
        update();
        active = false;
    }
    public void restart(double elbowPwr, double extendABobPwr){
        setElbowPwr(elbowPwr);
        setExtendABobPwr(extendABobPwr);
        active = true;
    }

    public void setActive(boolean active){this.active = active;}



    public void resetEncoders() {
        //just encoders - only safe to call if we know collector is in normal starting position
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABob.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public boolean extendToMin(){
        return extendToMin(extendABobPwr, 15);
    }
    public boolean extendToMin(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendMin);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToLow(){
        return extendToLow(extendABobPwr, 15);
    }
    public boolean extendToLow(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendLow);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToMid(){
        return extendToMid(extendABobPwr, 15);
    }
    public boolean extendToMid(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendMid);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToPosition(int position, double speed){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(position);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<15){
            return true;
        }
        return false;
    }
    public boolean extendToPositionUnsafe(int position, double speed){
        setExtendABobPwr(speed);
        setExtendABobTargetPosUnsafeReally(position);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<15){
            return true;
        }
        return false;
    }
    public boolean extendToMax(){
        return extendToMax(extendABobPwr, 15);
    }
    public boolean extendToMax(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendMax);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }

    public boolean extendToReverseDeposit(){
        return extendToMax(extendABobPwr, 15);
    }
    public boolean extendToReverseDeposit(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendDeposit);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }

    public boolean nearTargetExtend(){
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<15) return true;
        else return false;
    }
    public boolean nearTargetElbow(){
        if ((Math.abs( getElbowCurrentPos()-getElbowTargetPos()))<55) return true;
        else return false;
    }
    public boolean nearTarget(){
        if (nearTargetElbow() && nearTargetExtend()) return true;
        else return false;
    }

    public void increaseElbowAngle(){
        setElbowTargetPos(Math.min(getElbowCurrentPos() + 100, pos_Intake));
    }

    public void adjustElbowAngle(double speed){
        if(extendABob.getCurrentPosition() > 600 && speed < 0 && elbow.getCurrentPosition() > 290)
            speed *= .8;
        if(extendABob.getCurrentPosition() > 1000 && speed < 0 && elbow.getCurrentPosition() > 290)
            speed *= .5;
        setElbowTargetPos(getElbowCurrentPos() + (int)(150 * speed));


    }

    public void adjustElbowAngleNoCap(double speed){
        setElbowTargetPosNoCap(getElbowCurrentPos() + (int)(200 * speed));
    }
    public void decreaseElbowAngle(){
        setElbowTargetPos(Math.max(getElbowCurrentPos() - 100, 0));
    }

    public void extendBelt(){
        setExtendABobTargetPos(Math.min(getExtendABobCurrentPos() + 100, extendMax));
    }

    public void retractBelt(){
        setExtendABobTargetPos(Math.max(getExtendABobCurrentPos() - 100, extendMin));
    }

    public void adjustBelt(double speed){
        setExtendABobTargetPos(getExtendABobCurrentPos() + (int)(200 * speed));
    }

    public void adjustBeltNoCap(double speed){
        setExtendABobTargetPosUnsafeReally(getExtendABobCurrentPos() + (int)(250 * speed));
    }

    public void runToAngle(double angle){
        setElbowTargetPos((int)(angle * ticksPerDegree));
    }//untested


}

