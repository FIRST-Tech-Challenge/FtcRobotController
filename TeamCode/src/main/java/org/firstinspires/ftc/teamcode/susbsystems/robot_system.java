package org.firstinspires.ftc.teamcode.susbsystems;

import static org.firstinspires.ftc.teamcode.Constants.EXTENSION_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.SHOULDER_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.SHOULDER_MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.TAG_ID_1_Y_MAX;
import static org.firstinspires.ftc.teamcode.Constants.TAG_ID_1_Y_MIN;
import static org.firstinspires.ftc.teamcode.Constants.TAG_ID_1_Z_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.TURRENT_OFFSET_135_X_Y;
import static org.firstinspires.ftc.teamcode.Constants.TURRENT_OFFSET_180_X_Y;
import static org.firstinspires.ftc.teamcode.Constants.TURRENT_OFFSET_225_X_Y;
import static org.firstinspires.ftc.teamcode.Constants.TURRENT_OFFSET_45_X_Y;
import static org.firstinspires.ftc.teamcode.Constants.TURRENT_OFFSET_90_X_Y;
import static org.firstinspires.ftc.teamcode.Constants.TURRENT_TO_CAMERA;
import static org.firstinspires.ftc.teamcode.Constants.WRIST_X_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.WRIST_Z_OFFSET;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Constants;

public class robot_system {
    // robot constants
    private double y_target_offset = 0;
    private double x_target_offset = 0;

    //robot subsystems
    Drive robot_drive;
    RelayDevice fan;
    RelayDevice fogger;
    Pump_Subsystem pump;
    Arm725 shoulder;
    LinkageExtension725 extension;
    Turret725 turret;
    Wrist wrist;

    //vision
    Vision725 vision;

    // turret variables
    int target_degrees = 135;
    public enum Subsystems {
        DRIVE, SHOULDER, TURRET, WRIST, PUMP, FAN, FOGGER, EXTENSION
    }

    //Vision variables
    Position tagLocation = new Position(DistanceUnit.INCH,0,0,0,0);
    boolean tagDetected = false;
    int tagID = 0;
    boolean treatmentTargetLocked = false;
    Position lockedTargetPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);

    //ready to treat
    boolean isReadyToTreat = false;
    boolean armLocationLogicImprovement = false;

    //fog cycle
    //****************************************************************************************************************************
    //****************************************************************************************************************************
    //fog Cycle variables
    private double pumpTime = 5;
    private double fogTime = 2;
    private double fanTime = 1;
    private int cycleTarget = 10;

    //fog cycle variables
    //********************
    private ElapsedTime fogRunTime = new ElapsedTime();
    private ElapsedTime fanRunTime = new ElapsedTime();
    private ElapsedTime pumpRunTime = new ElapsedTime();
    boolean cycling = false;
    int cyclecount = 0;

    //armToOrientation
    //*************************************************************************************************************************
    //*************************************************************************************************************************
    //armToOrientation Constants
    private static final double DEFAULT_WRIST_ANGLE = 75;

    //armToOrientation variables
    private Orientation target_angle;
    private Position target_position;
    private Orientation current_angle;
    private Position arm_position;
    private Orientation robot_orientation;
    private boolean arm_automation = false;
    private int arm_state = 0;
    private double arm_x = 0;
    private double arm_y = 0;
    private double arm_z = 0;
    private boolean arm_ready = false;
    private boolean arm_is_busy = false;
    private boolean arm_homing = false;
    private boolean arm_homed = false;
    private boolean arm_last_position_bad = false;
    //buttons
    // extension info
    private double extension_target = 0;
    //local variables
    int runForTime = 0; //seconds
    int runForTicks = 0; //encoder ticks

    //full cycle
    //*************************************************************************************************************************
    //*************************************************************************************************************************
    private boolean fullCycleAutomation = false;
    private int fullCycleState = 0;

    public robot_system(HardwareMap hm) {
        pump = new Pump_Subsystem(hm, "pump");
        shoulder = new Arm725(hm);
        turret = new Turret725(hm);
        wrist = new Wrist(hm);
        fan = new RelayDevice(hm, "valve1");
        fogger = new RelayDevice(hm, "compressor1");
        extension = new LinkageExtension725(hm);
        robot_drive = new Drive(hm);
        vision = new Vision725(hm);
        init();
    }

    private void init() {
        extension.StartHome();
        turret.StartHome();
        arm_position = new Position(DistanceUnit.INCH, 0, 0, 0, System.nanoTime());
        target_position = new Position(DistanceUnit.INCH, 0, 0, 0, System.nanoTime());
        robot_orientation = new Orientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 0, 0, 0, System.nanoTime());
        arm_ready = false;
    }

    public void update() {
        pump.update();
        shoulder.Update();
        turret.Update();
        wrist.Update();
        fan.Update();
        fogger.Update();
        extension.Update();
        vision.Update();
        updatearm();
        arm_is_busy = shoulder.IsBusy() || turret.IsBusy() || wrist.IsBusy() || extension.IsBusy();
        if (cycling) {cycle();}
        if (arm_automation) {armToPosition();}
        if (!arm_ready) {armReady();}
        if (fullCycleAutomation) {fullCycle();}

        if (!treatmentTargetLocked) {
            if (vision.IsFilteredDataReady() && vision.isTagDetected()) {
                tagLocation = vision.GetFilteredPos();
            } else {
                tagLocation = vision.GetPos(); // fallback to raw during startup
            }
            tagDetected = vision.isTagDetected();
            tagID = vision.getTagID();
            if (arm_homed) {isReadyToTreat = tagToArmTest();}
        } else {
            tagLocation = lockedTargetPosition;
            tagDetected = true;
            // Maintain the locked target until the current task finishes or is manually released
        }
    }

    public void robot_drive(double power, double steering) {
        robot_drive.Set(power, steering);
    }

    public void doSomething(Subsystems subsystem) {
        switch (subsystem) {
            case PUMP:
                pump.ToggleState();
                break;
            case FAN:
                fan.ToggleState();
            default:
                break;
        }
    }

    public void runForTime(int seconds) {
        runForTime = seconds;
    }

    public void runForTicks(int ticks) {
        runForTicks = ticks;
    }

    //code used for the automated fog cycle
    //************************************
    //*************************************
    public void startFogCycle() {
        cycling = true;
        init_cycle();
    }

    public void stopFogCycle() {
        cycling = false;
        pump.TurnOff();
        fan.TurnOff();
        cyclecount = 0;
    }

    private void init_cycle() {
        cycling = true;
        pumpRunTime.reset();
        fogRunTime.reset();
        cyclecount = 0;
        pump.TurnOn();
        fogger.TurnOn();
    }

    private void cycle() {
        if (pumpRunTime.seconds() > pumpTime) {
            pump.TurnOff();
        }
        if (cyclecount == 0 && fogRunTime.seconds() > fogTime) {
            fanRunTime.reset();
            fan.TurnOn();
            cyclecount++;
        }
        if (fanRunTime.seconds() > fanTime) {
            fan.ToggleState();
            fanRunTime.reset();
        }
        if (cyclecount > 0) {
            if (fogRunTime.seconds() > fogTime) {
                fogger.ToggleState();
                fogRunTime.reset();
                cyclecount++;
            }
        }
        if (cyclecount > cycleTarget) {
            pump.TurnOff();
            fan.TurnOff();
            cycling = false;
        }
    }

    public int getCyclecount() {return cyclecount;}
    public boolean isCycling() {return cycling;}
    public double getFogTime(){return fogTime;}
    public double getFanTime(){return fanTime;}
    public double getPumpTime(){return pumpTime;}
    public int getCycleTarget(){return cycleTarget;}
    public void setPumpTime(double pump){
        if (pump < 1) pump = 1;
        if (pump > 20) pump = 20;
        pumpTime = pump;}
    public void setFanTime(double fan){
        if (fan <1) fan = 1;
        if (fan > 10) fan = 10;
        fanTime = fan;}
    public void setFogTime(double fog){
        if (fog <1) fog = 1;
        if (fog > 10) fog = 10;
        fogTime = fog;}
    public void setCyclecount(int count){
        if (count < 5) count = 5;
        if (count > 30) count = 30;
        cycleTarget = count;}

    //code used for the move arm to point in space
    //************************************
    //*************************************

    //update arm current position
    private void updatearm() {
        // x left right of robot, right is positive.  (arm is on the front of the robot, electronics are on the back)
        // y forward / backward from robot - forward is positive
        // z up is positive
        // y, x, z 0,0,0 is camera zip tied to robot frame
        arm_position.z = shoulder.GetHeight() + TURRENT_TO_CAMERA.z; //7.5 is the height of shoulder rotation above the camera
        arm_position.y = turret.getPosition().y + TURRENT_TO_CAMERA.y; //4.5 is how far the end of the arm is in front of the camera
        arm_position.x = -(extension.GetPos() + shoulder.getExtension() - turret.getPosition().x)+TURRENT_TO_CAMERA.x+10; // -8 for camera adjustment added 10 for no clear reason
        if (arm_automation) {
            if (!shoulder.IsBusy() && !turret.IsBusy() && !extension.IsBusy() && arm_state == 0) {
                arm_automation = false;
            }
        }

        if(shoulder.GetRawPos() <25 && shoulder.GetRawPos() > -25 && turret.GetRawPos() >.310 && extension.isAtHome()){
            //arm_homing = false;
            if(arm_ready){arm_homed = true;}
        }
        else arm_homed = false;
    }

    public void startarmToPosition(Position target) {
        if(!arm_is_busy){
            //arm_automation = true;
            arm_last_position_bad = init_armToPosition(target, target_degrees);
        }
    }

    public void armHome() {
        //arm_automation = true;
        stopArmToPosition();
        init_armToHome();
    }

    private void armReady() {
        if (shoulder.Homed() && turret.Homed() && extension.isAtHome()) {
            arm_ready = true;
        }
    }

    public void stopArmToPosition() {
        arm_automation = false;
        arm_homing = false;
        arm_state = 0;
        treatmentTargetLocked = false;
        wrist.Stop();
        turret.Stop();
        shoulder.Stop();
        turret.Stop();
        extension.Stop();
    }

    public void lockCurrentFilteredTarget() {
        if (vision.IsFilteredDataReady() && vision.isTagDetected()) {
            lockedTargetPosition = vision.GetFilteredPos();
            treatmentTargetLocked = true;
        }
    }

    public void unlockTreatmentTarget() {
        treatmentTargetLocked = false;
    }

    public boolean isTreatmentTargetLocked() {
        return treatmentTargetLocked;
    }

    private void init_armToHome() {
        if(arm_ready && !arm_is_busy) {
            arm_automation = true;
            arm_homing = true;
            extension.StartHome();
            wrist.SetPos(.9);
            arm_state = 1;
        }
    }
/*
0 (5 y , -7 x)
90 (15 y , -20 x)
135 (13 y, -24x)
180 (4 y, -28x)
225 (-3y, -24x)
*/
    private boolean init_armToPosition(Position target, int degrees) {
        boolean valid_position = false;
        if (arm_ready&&!arm_is_busy&& arm_homed) {
            arm_state = 1;
            arm_automation = true;
            if(target.x > -20) stopArmToPosition();
            if(target.x < -38) stopArmToPosition();
            if(target.y > 15) stopArmToPosition();
            if(target.y <-3) stopArmToPosition();
            if(target.z <SHOULDER_MIN_HEIGHT) stopArmToPosition();
            if(target.z > SHOULDER_MAX_HEIGHT) stopArmToPosition();
            if(arm_automation){
                if (degrees == 180) {
                    turret.GoTo(Constants.ONEEIGHTY_DEGREES);
                    valid_position = true;
                } else if (degrees == 90) {
                    turret.GoTo(Constants.NINETY_DEGREES);
                    valid_position = true;
                } else if (degrees == 135) {
                    turret.GoTo(Constants.ONETHIRTYFIVE_DEGREES);
                    valid_position = true;
                } else if (degrees == 225) {
                    turret.GoTo(Constants.TWOTWENTYFIVE_DEGREES);
                    valid_position = true;
                } else if (degrees == 45) {
                    turret.GoTo(Constants.FORTYFIVE_DEGREES);
                    valid_position = true;
                } else {stopArmToPosition();}
            }
        }
        return valid_position;
    }

    private void armToPosition() {
        if(!arm_is_busy&&!arm_homing) {
            if (arm_state == 1 &&!turret.IsBusy()) {
                arm_state = 2;
                extension.GoTo(2);
                wrist.SetTargetPos(.9);
            }
            if (arm_state == 2 && !extension.IsBusy()) {
                arm_state = 3;
                shoulder.GoToHeight(target_position.z - TURRENT_TO_CAMERA.z);
            }
            if (arm_state == 3 && !shoulder.IsBusy()) {
                arm_state = 4;
                //wrist.SetLevel(shoulder.GetPos());
                wrist.SetTargetPos(.55);
            }
            if (arm_state == 4 && !wrist.IsBusy()) {
                arm_state = 5;
                extension_target = Math.abs(target_position.x)-(Math.abs(WRIST_X_OFFSET)+Math.abs(arm_position.x)+1);
                extension.GoTo(extension_target);
            }
            if (arm_state == 5 && !extension.IsBusy()) {
                arm_state = 0;
                arm_automation = false;
                treatmentTargetLocked = false;
            }
        }
        else if(!arm_is_busy && arm_homing){
            if (arm_state == 1 &&!extension.IsBusy()) {
                arm_state = 2;
                shoulder.GoToEncoderPosition(0);
                shoulder.Update();
            }
            if (arm_state == 2 && !shoulder.IsBusy()) {
                arm_state = 3;
                turret.GoTo(Constants.TURRET_MAX_POSITION);
            }
            if (arm_state == 3 && !turret.IsBusy()) {
                arm_state = 0;
                arm_automation = false;
                arm_homing = false;
            }
        }
    }

    public boolean isArm_automation() {return arm_automation;}
    public boolean isArm_ready() {return arm_ready;}
    public int armAutoState() {return arm_state;}
    public Position getCurrent_Arm_Position() {return arm_position;}
    public boolean isArmBusy(){return arm_is_busy;}
    public boolean isArmHomed(){return arm_homed;}
    public boolean isArm_last_position_bad(){return arm_last_position_bad;}
    public Position getTarget_position(){return target_position;}

    //shoulder functions
    //**************************************************************************************
    //**************************************************************************************
    public void shoulderHome() {shoulder.Home();}

    public void shoulderSetPower(double power) {shoulder.SetPower(power);}

    public void shoulderStop() {shoulder.Stop();}

    public boolean shoulderIsBusy() {return shoulder.IsBusy();}
    public double shoulderHeight(){return shoulder.GetHeight();}
    public double shoulderLength(){return shoulder.getExtension();}
    public double shoulderRaw(){return shoulder.GetRawPos();}
    public boolean shoulderIsHomed(){
        return shoulder.GetRawPos() < 25 && shoulder.GetRawPos() > -25;
    }
    //turret functions
    //**************************************************************************************
    //**************************************************************************************
    public boolean turretIsBusy(){return turret.IsBusy();}
    public boolean turretIsHomed(){return turret.Homed();}
    public double turretPosition(){return turret.GetRawPos();}
    public double turretTargetPosition(){return turret.GetTargetPosition();}
    public void moveTurretManual(double power){
        if (!turret.IsBusy()) turret.SetPower(power);
        else if (Math.abs(power) > 0) {
            turret.Stop();
            turret.SetPower(power);
        }
    }
    public void moveTurret(int degrees){
        if(!arm_is_busy) {
            if (degrees == 0) {
                turret.GoTo(Constants.ZERO_DEGREES);
            }
            if (degrees == 45) {
                turret.GoTo(Constants.FORTYFIVE_DEGREES);
            }
            if (degrees == 90) {
                turret.GoTo(Constants.NINETY_DEGREES);
            }
            if (degrees == 135) {
                turret.GoTo(Constants.ONETHIRTYFIVE_DEGREES);
            }
            if (degrees == 180) {
                turret.GoTo(Constants.ONEEIGHTY_DEGREES);
            }
            if (degrees == 225) {
                turret.GoTo(Constants.TWOTWENTYFIVE_DEGREES);
            }
        }
    }
    //extension functions
    //**************************************************************************************
    //**************************************************************************************
    public double getExtensionTarget() {return extension.GetTargetPos();}
    public double getExtensionLocal_target(){return extension_target;}

    public boolean isBusyExtension() {return extension.IsBusy();}
    public void moveExtensionManual(double power) {
        if (!extension.IsBusy()) extension.SetPower(power);
        else if (Math.abs(power) > 0) {
            extension.Stop();
            extension.SetPower(power);
        }
    }
    public boolean isExtensionHome(){return extension.isAtHome();}
    //wrist functions
    //**************************************************************************************
    //**************************************************************************************
    public double getWristHeight() {return wrist.GetHeight();}
    public double getWristLength() {return wrist.GetLength();}
    public double getWristCalc() {return wrist.GetCalcPos();}
    public boolean isWristBusy() {return wrist.IsBusy();}

    public void moveWristManual(double power) {
        if(!wrist.IsBusy()){
            if (power > 0) {wrist.SetPos(wrist.GetRawPos() + .001);}
            if (power < 0) {wrist.SetPos(wrist.GetRawPos() - .001);}
        }
    }
    public void wristGoPos(double position){
        wrist.SetTargetPos(position);
    }

    //armToPosition and Spray
    //**************************************************************************
    //**************************************************************************


    public void startFullCycle(Position target){
        if(arm_ready && arm_homed){
            fullCycleAutomation = true;
            fullCycleState = 1;
            arm_last_position_bad = init_armToPosition(target,target_degrees);
        }
    }

    public void stopFullCycle(){
        fullCycleAutomation = false;
        fullCycleState = 0;
        stopArmToPosition();
        stopFogCycle();
    }
    private void fullCycle(){
        if(fullCycleState == 1 && !arm_automation){
            fullCycleState++;
            init_cycle();
        }
        if(fullCycleState == 2 && !cycling){
            fullCycleState++;
            pump.TurnOff();
            fan.TurnOff();
            fogger.TurnOff();
            init_armToHome();
        }
        if(fullCycleState ==3 && !arm_automation){
            fullCycleState = 0;
            fullCycleAutomation = false;
        }
    }
    public int getFullCycleState(){return fullCycleState;}
    public boolean isFullCycleAutomation(){return fullCycleAutomation;}

    // vision code
    public Position getTagLocation(){return tagLocation;}
    public boolean isTagDetected(){return tagDetected;}

    // overall function info
    public boolean isReadyToTreat(){return isReadyToTreat;}
    public boolean isArmLocationLogicImprovement(){return armLocationLogicImprovement;}
    public double getTarget_degrees(){return target_degrees;}

    public void startTreatment(boolean arm_or_treat){
        if(isReadyToTreat&&arm_homed){
            //target_position = vision.GetPos();
            if(arm_or_treat)
                startFullCycle(target_position);
            else
                arm_last_position_bad = init_armToPosition(target_position, target_degrees);

        }
    }
    private boolean tagToArmTest(){
        boolean ready = false;
        //tag ID 1 is left
        if(tagID ==1 && arm_homed) {
            //target_position = tagLocation;
            target_position.z = tagLocation.z + TAG_ID_1_Z_OFFSET - WRIST_Z_OFFSET;
            if(target_position.z > SHOULDER_MAX_HEIGHT || target_position.z < SHOULDER_MIN_HEIGHT) {
                target_position.z = 0;
                return ready;
            }
            if(tagLocation.y >TURRENT_OFFSET_90_X_Y.y+TURRENT_TO_CAMERA.y+TAG_ID_1_Y_MAX) {
                target_position.y = 0;
                return ready;
            }
            if(tagLocation.y<TURRENT_OFFSET_225_X_Y.y+TURRENT_TO_CAMERA.y+TAG_ID_1_Y_MIN) {
                target_position.y =0;
                return ready;
            }
            target_position.y = tagLocation.y;
            if(tagLocation.x > -30 ) {
                target_position.x = 0;
                return ready;
            }
            if(tagLocation.x < -44) {
                target_position.x = 0;
                return ready;
            }
            target_position.x = tagLocation.x;
            armLocationLogicImprovement = false;
            //if it gets here then it should work or I have my bounds checking wrong.
            if(tagLocation.y < TURRENT_OFFSET_90_X_Y.y+TURRENT_TO_CAMERA.y+TAG_ID_1_Y_MAX && tagLocation.y > TURRENT_OFFSET_135_X_Y.y+TURRENT_TO_CAMERA.y+TAG_ID_1_Y_MAX){
                ready = true;
                target_degrees = 90;
                target_position.y = TURRENT_OFFSET_90_X_Y.y+TURRENT_TO_CAMERA.y;
                target_position.x = tagLocation.x+2;
            }
            if(tagLocation.y < TAG_ID_1_Y_MAX+TURRENT_TO_CAMERA.y && tagLocation.y > TAG_ID_1_Y_MIN+TURRENT_TO_CAMERA.y && tagLocation.x <TURRENT_TO_CAMERA.x + TURRENT_OFFSET_180_X_Y.x){
                ready = true;
                target_degrees = 180;
                target_position.y = TURRENT_OFFSET_180_X_Y.y+TURRENT_TO_CAMERA.y;
                target_position.x = tagLocation.x+2;
            }
            if(tagLocation.y < TURRENT_OFFSET_135_X_Y.y+TURRENT_TO_CAMERA.y+TAG_ID_1_Y_MAX && tagLocation.y > TURRENT_OFFSET_180_X_Y.y+TURRENT_TO_CAMERA.y+TAG_ID_1_Y_MAX){
                ready = true;
                target_degrees = 135;
                //target_position.z = target_position.z +3;  //adjust for arm drop or something
                target_position.y = TURRENT_OFFSET_135_X_Y.y+TURRENT_TO_CAMERA.y;
                target_position.x = tagLocation.x+2;
            }
            if(tagLocation.y < TURRENT_OFFSET_180_X_Y.y+TURRENT_TO_CAMERA.y+TAG_ID_1_Y_MAX && tagLocation.y > TURRENT_OFFSET_225_X_Y.y+TURRENT_TO_CAMERA.y+TAG_ID_1_Y_MAX){
                ready = true;
                target_degrees = 225;
                target_position.y = TURRENT_OFFSET_135_X_Y.y+TURRENT_TO_CAMERA.y;
                target_position.x = tagLocation.x+2;
            }
            if(!ready) armLocationLogicImprovement = true;
        }
        return ready;
    }
}
