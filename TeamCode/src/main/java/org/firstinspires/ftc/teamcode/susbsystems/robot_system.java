package org.firstinspires.ftc.teamcode.susbsystems;

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
    ArmAutomation armAutomationController;
    InverseKinematics inverseKinematics;

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
    Orientation target_angle;
    Position target_position;
    Orientation current_angle;
    Position arm_position;
    Orientation robot_orientation;
    boolean arm_automation = false;
    int arm_state = 0;
    double arm_x = 0;
    double arm_y = 0;
    double arm_z = 0;
    boolean arm_ready = false;
    boolean arm_is_busy = false;
    boolean arm_homing = false;
    boolean arm_homed = false;
    boolean arm_last_position_bad = false;
    //buttons
    // extension info
    double extension_target = 0;
    //local variables
    int runForTime = 0; //seconds
    int runForTicks = 0; //encoder ticks

    //full cycle
    //*************************************************************************************************************************
    //*************************************************************************************************************************
    boolean fullCycleAutomation = false;
    int fullCycleState = 0;

    // === TESTING ONLY ===
    // Lock-triggered FK validation and simple movement automation for Phase 1.
    boolean testAutoMoveActive = false;
    int testAutoMovePhase = 0;
    double testTurretTargetDegrees = 135;
    double testExtensionTarget = 0;
    double testShoulderTargetAngle = 0;
    double testWristTargetAngle = 0;
    IKSolution lastIKSolution = new IKSolution();
    private Position computedEndEffectorPose = new Position(DistanceUnit.INCH, 0, 0, 0, System.nanoTime());

    public static class IKSolution {
        public boolean reachable;
        public double turretDegrees;
        public double turretPotTarget;
        public double shoulderAngle;
        public double extensionLength;
        public double wristAngle;
        public String failureReason;
        public double acceptedYMin;
        public double acceptedYMax;
        public double selectionScore;
        public int candidateCount;
        public double targetX;
        public double targetY;
        public double targetZ;
        public double turretOffsetX;
        public double turretOffsetY;
        public double wristLength;
        public double wristHeight;
        public double shoulderHeight;
        public double shoulderLength;
        public double rawExtensionLength;
        public double targetErrorY;
        public double targetErrorZ;
    }

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
        armAutomationController = new ArmAutomation(this);
        inverseKinematics = new InverseKinematics();
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
        armAutomationController.updateArmPositionAndHome();
        arm_is_busy = shoulder.IsBusy() || turret.IsBusy() || wrist.IsBusy() || extension.IsBusy();
        if (cycling) {cycle();}
        if (arm_automation) {armAutomationController.armToPosition();}
        if (!arm_ready) {armAutomationController.armReady();}
        if (fullCycleAutomation) {armAutomationController.fullCycle();}
        computeForwardKinematics();
        if (testAutoMoveActive) {armAutomationController.testLockedTargetAutoMove();}

        if (!treatmentTargetLocked) {
            if (vision.IsFilteredDataReady() && vision.isTagDetected()) {
                tagLocation = vision.GetFilteredPos();
            } else {
                tagLocation = vision.GetPos(); // fallback to raw during startup
            }
            tagDetected = vision.isTagDetected();
            tagID = vision.getTagID();
            if (arm_homed) {isReadyToTreat = armAutomationController.tagToArmTest();}
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

    void init_cycle() {
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

    private Position getAutomationTurretOffset() {
        return RobotGeometry.turretOffset(turret.getNewcurrentAngleDegrees());
    }

    private Position getAutomationTurretOffset(double turretDegrees) {
        return RobotGeometry.turretOffset(turretDegrees);
    }

    private double getAutomationTurretTargetY(double turretDegrees) {
        return RobotGeometry.turretTargetY(turretDegrees);
    }

    private double getAutomationTurretTargetX(double turretDegrees) {
        return RobotGeometry.turretTargetX(turretDegrees);
    }

    public void startarmToPosition(Position target) {
        if(!arm_is_busy){
            //arm_automation = true;
            arm_last_position_bad = armAutomationController.initArmToPosition(target, target_degrees);
        }
    }

    public void armHome() {
        //arm_automation = true;
        stopArmToPosition();
        armAutomationController.initArmToHome();
    }

    public void stopArmToPosition() {
        armAutomationController.stopArmToPosition();
    }

    public void lockCurrentFilteredTarget() {
        lockCurrentFilteredTarget(false);
    }

    public void lockCurrentFilteredTarget(boolean shouldStartTestAutoMove) {
        if (vision.IsFilteredDataReady() && vision.isTagDetected()) {
            // Correct the locked IK target only; keep raw vision telemetry unchanged for measurement checks.
            lockedTargetPosition = RobotGeometry.correctedIKTarget(vision.GetFilteredPos());
            treatmentTargetLocked = true;
            if (shouldStartTestAutoMove) {
                armAutomationController.initTestLockedTargetAutoMove();
            }
        }
    }

    public void unlockTreatmentTarget() {
        treatmentTargetLocked = false;
        testStopAutoMove();
    }

    public boolean isTreatmentTargetLocked() {
        return treatmentTargetLocked;
    }

    public double getIKCameraXCorrection() {
        return RobotGeometry.TEST_IK_CAMERA_X_CORRECTION;
    }

    // === TESTING ONLY ===
    private void computeForwardKinematics() {
        computedEndEffectorPose = ForwardKinematics.computeEndEffectorPose(
                extension.GetPos(),
                shoulder.getExtension(),
                shoulder.GetHeight(),
                RobotGeometry.levelWristLength(),
                RobotGeometry.levelWristHeight(),
                getAutomationTurretOffset());
    }

    public Position getComputedEndEffectorPose() {
        return computedEndEffectorPose;
    }

    public boolean isTestAutoMoveActive() {
        return testAutoMoveActive;
    }

    public int getTestAutoMovePhase() {
        return testAutoMovePhase;
    }

    public IKSolution getLastIKSolution() {
        return lastIKSolution;
    }

    public double getTestTurretTargetDegrees() {
        return testTurretTargetDegrees;
    }

    public double getTestTurretTargetPot() {
        return TurretKinematics.potForDegrees(testTurretTargetDegrees);
    }

    public double getTestExtensionTarget() {
        return testExtensionTarget;
    }

    public double getTestShoulderTargetAngle() {
        return testShoulderTargetAngle;
    }

    public double getTestWristTargetAngle() {
        return testWristTargetAngle;
    }

    // === TESTING ONLY ===
    // Compute-only IK solver. This does not command motors or modify automation state.
    public IKSolution solveIK(Position target) {
        return inverseKinematics.solve(target);
    }

    public IKSolution testIK(Position target) {
        return solveIK(target);
    }

    public void testStopAutoMove() {
        armAutomationController.testStopAutoMove();
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
    public double shoulderAngle(){return shoulder.GetPos();}
    public double shoulderLength(){return shoulder.getExtension();}
    public double shoulderRaw(){return shoulder.GetRawPos();}
    public double shoulderTargetAngle(){return shoulder.GetTargetPos();}
    public double shoulderRawTarget(){return shoulder.GetRawTargetPos();}
    public double shoulderTargetErrorDegrees(){return shoulder.GetTargetErrorDegrees();}
    public double shoulderTargetErrorTicks(){return shoulder.GetTargetErrorTicks();}
    public double shoulderPower(){return shoulder.GetPower();}
    public boolean shoulderIsHomed(){
        return shoulder.GetRawPos() < 25 && shoulder.GetRawPos() > -25;
    }
    //turret functions
    //**************************************************************************************
    //**************************************************************************************
    public boolean turretIsBusy(){return turret.IsBusy();}
    public boolean turretIsHomed(){return turret.Homed();}
    public double turretAngle(){return turret.getNewcurrentAngleDegrees();}
    public double turretPosition(){return turret.GetRawPos();}
    public double turretTargetPosition(){return turret.GetTargetPosition();}
    public double shoulderPivotX(){return getAutomationTurretOffset().x;}
    public double shoulderPivotY(){return getAutomationTurretOffset().y;}
    public double shoulderPivotRadius(){
        Position turretOffset = getAutomationTurretOffset();
        return Math.sqrt((turretOffset.x * turretOffset.x) + (turretOffset.y * turretOffset.y));
    }
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
    public double getExtensionPosition() {return extension.GetPos();}
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
    public double getLevelWristHeight() {return RobotGeometry.levelWristHeight();}
    public double getLevelWristLength() {return RobotGeometry.levelWristLength();}
    public double getWristAngle() {return wrist.GetAngle();}
    public double getWristCalc() {return wrist.GetCalcPos();}
    public double getWristTargetAngle() {return wrist.GetTargetPos();}
    public double getWristRawTargetPosition() {return wrist.GetRawTargetPos();}
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
        armAutomationController.startFullCycle(target);
    }

    public void stopFullCycle(){
        armAutomationController.stopFullCycle();
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
        armAutomationController.startTreatment(arm_or_treat);
    }
}
