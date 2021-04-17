package global;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

import globalfunctions.Constants;
import globalfunctions.Optimizer;
import globalfunctions.TerraThread;
import telefunctions.AutoModule;
import telefunctions.ButtonController;
import telefunctions.Cycle;
import telefunctions.Limits;
import util.Stage;
import util.CodeSeg;

public class TerraBot {

    public DcMotor r1;
    public DcMotor l1;
    public DcMotor r2;
    public DcMotor l2;
    public DcMotorEx outr;
    public DcMotorEx outl;
    public DcMotor arm;


    public DcMotor in;

    public CRServo rh;
    public CRServo rh2;

    public CRServo wge;
    public Rev2mDistanceSensor wgp;

    public Servo cll;
    public Servo clr;
    public Servo rp;

//
    public Cycle pushControl = new Cycle(0.1, 0.25, 0.4);
//    public Cycle pushControl = new Cycle(0.1, 0.27, 0.25);

    public Cycle cllControl = new Cycle(0.2, 0.5, 1);
    public Cycle clrControl = new Cycle(1, 0.5, 0.0);
//    public Cycle cllControl = new Cycle(0.2, 1);
//    public Cycle clrControl = new Cycle(1, 0.0);


    public AutoAimer autoAimer = new AutoAimer();

    public AngularPosition angularPosition = new AngularPosition();

    public Localizer localizer = new Localizer();

    public Limits limits = new Limits();

    public boolean intaking = false;
    public boolean outtaking = false;



    public boolean fastMode = false;
    public boolean wgeStartMode = true;
    public int wgStartMode = 0;

    public AutoModule shooter = new AutoModule(); // 0
    public AutoModule aimer = new AutoModule();
    public AutoModule wobbleGoal = new AutoModule();
    public AutoModule powerShot = new AutoModule();

    public ArrayList<AutoModule> autoModules = new ArrayList<>();

    public ButtonController outtakeButtonController = new ButtonController();
    public ButtonController fastModeController = new ButtonController();
    public ButtonController powerShotController = new ButtonController();

    public Odometry odometry = new Odometry();

    public TerraThread odometryThread;

    public ElapsedTime odometryTime = new ElapsedTime();

    public double[] aimerPos = Constants.TELE_START;

    public boolean isMovementAvailable = true;

    public boolean powershotMode = false;




    public void init(HardwareMap hwMap) {

        l1 = getMotor(hwMap, "l1", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2 = getMotor(hwMap, "l2", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1 = getMotor(hwMap, "r1", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2 = getMotor(hwMap, "r2", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        in = getMotor(hwMap, "in", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outr = getMotorEx(hwMap, "outr", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outl = getMotorEx(hwMap, "outl", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm = getMotor(hwMap, "arm", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        outr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rh = getCRServo(hwMap, "rh", CRServo.Direction.FORWARD);
        rh2 = getCRServo(hwMap, "rh2", CRServo.Direction.FORWARD);
        cll = getServo(hwMap, "cll", Servo.Direction.FORWARD, Constants.CLL_GRAB);
        clr = getServo(hwMap, "clr", Servo.Direction.REVERSE, Constants.CLL_OPEN);
        rp = getServo(hwMap, "rp", Servo.Direction.FORWARD, Constants.RP_START);
        wge = getCRServo(hwMap, "wge", CRServo.Direction.REVERSE);

        wgp = hwMap.get(Rev2mDistanceSensor.class, "wgp");

        angularPosition.init(hwMap);
        localizer.init(hwMap);

        odometry.updateEncoderPositions(getLeftOdo(), getCenterOdo(), getRightOdo());

        limits.addLimit(arm, Constants.WG_LOWER_LIMIT, Constants.WG_UPPER_LIMIT);
        limits.addLimit(wge, 0, Constants.WGE_UPPER_LIMIT);



        outr.setVelocityPIDFCoefficients(54, 0, 0, 14);
        outl.setVelocityPIDFCoefficients(54, 0, 0, 14);
//
//        outr.setVelocityPIDFCoefficients(1, 1, 10, 20);
//        outl.setVelocityPIDFCoefficients(100, 1, 10, 20);



        defineShooter();
        defineAimer();
        defineWobbleGoal();
    }

    public DcMotor getMotor(HardwareMap hwMap, String name, DcMotor.Direction dir, DcMotor.ZeroPowerBehavior zpb, DcMotor.RunMode mode){
        DcMotor dcMotor = hwMap.get(DcMotor.class, name);
        dcMotor.setPower(0);
        dcMotor.setDirection(dir);
        dcMotor.setZeroPowerBehavior(zpb);
        dcMotor.setMode(mode);

        return dcMotor;
    }

    public DcMotorEx getMotorEx(HardwareMap hwMap, String name, DcMotor.Direction dir, DcMotor.ZeroPowerBehavior zpb, DcMotor.RunMode mode){
        DcMotorEx dcMotor = hwMap.get(DcMotorEx.class, name);
        dcMotor.setPower(0);
        dcMotor.setDirection(dir);
        dcMotor.setZeroPowerBehavior(zpb);
        dcMotor.setMode(mode);

        return dcMotor;
    }

    public Servo getServo(HardwareMap hwMap, String name, Servo.Direction dir, double startpos){
        Servo servo = hwMap.get(Servo.class, name);
        servo.setDirection(dir);
        servo.setPosition(startpos);
        return servo;
    }

    public CRServo getCRServo(HardwareMap hwMap, String name, CRServo.Direction dir){
        CRServo crServo = hwMap.get(CRServo.class, name);
        crServo.setPower(0);
        crServo.setDirection(dir);
        return crServo;
    }


    public void move(double f, double s, double t){
        l1.setPower(f+s-t);
        l2.setPower(-f+s+t);
        r1.setPower(f-s+t);
        r2.setPower(-f-s-t);
    }

    public void intake(double p){
        in.setPower(p);
        rh.setPower(p);
        rh2.setPower(p);
    }

    public void pushRings(double pos){
        rp.setPosition(pos);
    }


    public void outtake(double p){
        outr.setPower(p);
        outl.setPower(p);
    }

    public void claw(double posLeft, double posRight){
        cll.setPosition(posLeft);
        clr.setPosition(posRight);

    }

//    public void openClaw() {
//        claw(Constants.CLL_OPEN, Constants.CLR_OPEN);
//    }
//
//    public void closeClaw() {
//        claw(Constants.CLL_GRAB, Constants.CLR_GRAB);
//    }

    public void updateIntake(boolean left_bumper, boolean right_bumper) {
        if(right_bumper){
            intaking = true;
        }else if(left_bumper){
            intaking = false;
            intake(-0.5);
        }else if(intaking){
            intake(1);
        }else{
            intake(0);
            if (!outtaking) { rh.setPower(0); }
        }
    }

    public void moveTeleOp(double f, double s, double t, double rt){
        if(isMovementAvailable){
            if (fastMode) {
                move(Math.signum(f) * Math.pow(Math.abs(f), 0.5), Math.signum(s) * Math.pow(Math.abs(s), 0.5), Math.signum(t) * Math.pow(Math.abs(t), 0.5));
            } else {
                move(0.5 * Math.signum(f) * Math.pow(Math.abs(f), 0.5), 0.5 * Math.signum(s) * Math.pow(Math.abs(s), 0.5), 0.4 * Math.signum(t) * Math.pow(Math.abs(t), 0.5));
            }
        }
        if (fastModeController.isPressing(rt > 0)) {
            fastMode = !fastMode;
            isMovementAvailable = true;
        }
    }

    public void moveArm(double p){
        if (isArmInLimits(p)) {
            arm.setPower(p + getRestPowArm());
        }
    }

    public double getRestPowArm(){
        return Constants.WG_REST_POW*Math.cos(Math.toRadians(getArmPos()));
    }

    public void moveArmWithEnc(double deg, double pow){
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition((int) (deg*Constants.NEV_DEGREES_TO_TICKS));
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveArm(Math.abs(pow) * Math.signum(deg - getArmPos()));
        while (arm.isBusy()){
            if (Math.abs(getArmPos() - deg) < 10) {
                moveArm(pow * 0.5);
            }
            if(isWgeInLimits(pow)) {
                updateWge();
            }else{
                wge.setPower(0);
            }
        }
        moveArm(0);
        wge.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveArmWithEncWithoutWGE(double deg, double pow){
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition((int) (deg*Constants.NEV_DEGREES_TO_TICKS));
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveArm(Math.abs(pow) * Math.signum(deg - getArmPos()));
        while (arm.isBusy()){
            if (Math.abs(getArmPos() - deg) < 10) {
                moveArm(pow * 0.5);
            }
        }
        moveArm(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setClawPos(int ind) {
        cllControl.changeCurr(ind);
        clrControl.changeCurr(ind);
        claw(cllControl.getPos(ind), clrControl.getPos(ind));
    }

    public double getArmPos(){
        return (arm.getCurrentPosition()/Constants.NEV_DEGREES_TO_TICKS);
    }

    public double getWgePos() { return wgp.getDistance(DistanceUnit.CM) - Constants.WGE_START; }

    public boolean isArmInLimits(double dir){
        return limits.isInLimits(arm, dir, getArmPos());
    }

    public boolean isWgeInLimits(double dir) { return limits.isInLimits(wge, dir, getWgePos()) && !Optimizer.inRange(getArmPos(), Constants.WGE_IGNORE_RANGE); }

    public double updateWge() {
        if (!wgeStartMode) {
            double wgePos = getWgePos();
            double armPos = getArmPos();
            double c = 0.05;
            double targetPos = (Constants.WGE_UPPER_LIMIT + Constants.WGE_ACC) - (Constants.WGE_UPPER_LIMIT + Constants.WGE_ACC) / (1 + Math.pow(Math.E, -c * (getArmPos() - 0.5 * Constants.WG_UPPER_LIMIT - Constants.WG_LOWER_LIMIT)));
            if (Math.abs(targetPos - wgePos) < Constants.WGE_ACC) {
                wge.setPower(0);
            } else {
                wge.setPower(Math.signum(targetPos - wgePos));
            }
            return targetPos;
        } else {
            wgeStartMode = getArmPos() < 40;
            return 0;
        }
    }

    public void controlWGE(double pos){
        if (!wgeStartMode) {
            double wgePos = getWgePos();
            double targetPos =  Constants.WGE_UPPER_LIMIT*pos;
            if (Math.abs(targetPos - wgePos) < Constants.WGE_ACC) {
                wge.setPower(0);
            } else {
                wge.setPower(Math.signum(targetPos - wgePos));
            }
        } else {
            wgeStartMode = getArmPos() < 40;
        }
    }
    public boolean isControlWgeDone(double pos){
        double targetPos =  Constants.WGE_UPPER_LIMIT*pos;
        return Math.abs(targetPos - getWgePos()) < Constants.WGE_ACC;
    }

    public boolean isWgeDone(){
        double c = 0.05;
        double targetPos = (Constants.WGE_UPPER_LIMIT + Constants.WGE_ACC) - (Constants.WGE_UPPER_LIMIT + Constants.WGE_ACC) / (1 + Math.pow(Math.E, -c * (getArmPos() - 0.5 * Constants.WG_UPPER_LIMIT - Constants.WG_LOWER_LIMIT)));
        return Math.abs(targetPos - getWgePos()) < Constants.WGE_ACC;
    }

    public boolean areAutomodulesRunning(){
        for (AutoModule a: autoModules) {
            if(a.inited) {
                if (a.isExecuting()) {
                    return true;
                }
            }
        }
        return false;
    }

    public void stopAllAutomodules(){
        for (AutoModule a: autoModules) {
            if(a.inited) {
                a.stop();
            }
        }
    }

    public void toggleOuttake(boolean in) {
        if (outtakeButtonController.isPressing(in)) {
            outtaking = !outtaking;
//            resetOuttake();
        }
    }

    public void updateRP(boolean lb, boolean rb){
        rp.setPosition(pushControl.update(lb, rb));
    }
    public void updateClaw(boolean dpl, boolean dpr){
        if(dpr) {
            claw(cllControl.getPos(0), clrControl.getPos(0));
        }else if(dpl){
            claw(cllControl.getPos(2), clrControl.getPos(2));
        }
    }

//    public void resetOuttake() {
////        autoAimer.update(odometry.getPos());
//        autoAimer.resetOuttake(getLeftAngPos(), getRightAngPos());
//    }

    public double getRightAngPos(){
        return (outr.getCurrentPosition()/Constants.GOBUILDA1_Ticks)*Constants.pi2;
    }

    public double getLeftAngPos(){
        return (outl.getCurrentPosition()/Constants.GOBUILDA1_Ticks)*Constants.pi2;
    }

    public double getRightAngVel(){
        return (outr.getVelocity()/Constants.GOBUILDA1_Ticks)*Constants.pi2;
    }

    public double getLeftAngVel(){
        return (outl.getVelocity()/Constants.GOBUILDA1_Ticks)*Constants.pi2;
    }

    public double[] getLocalizerPos() {
        return localizer.getPos();
    }

    public void updateLocalizer() {
        localizer.update(angularPosition.getHeadingGY());
    }

    public void setHeading(double angle) {
        localizer.update(angle);
        odometry.resetHeading(angle);
    }

    public void updateOdoWithSensors() {
        odometry.resetPos(getLocalizerPos());
    }


    public void outtakeWithCalculations() {
        if(outtaking){
            if(outr.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
                outr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                outl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(autoAimer.hasPosBeenUpdated()){
                autoAimer.updateTargetSpeed();
                outr.setVelocity(autoAimer.getOutrTargetVel());
                outl.setVelocity(autoAimer.getOutlTargetVel());
            }

        }else{
            if(outr.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER)){
                outr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                outl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(outr.getPower() != 0) {
                outr.setPower(0);
                outl.setPower(0);
            }
            if (!intaking) { rh.setPower(0); }
        }
    }

    public int getLeftOdo() {
        return r1.getCurrentPosition();
    }

    public int getRightOdo() {
        return l2.getCurrentPosition();
    }

    public int getCenterOdo() {
        return r2.getCurrentPosition();
    }

    public void updateOdometry() {
        odometry.updateGlobalPosition(getLeftOdo(), getCenterOdo(), getRightOdo());
    }
    public void optimizeHeading(){
        odometry.resetHeading(Optimizer.optimizeHeading(odometry.h));
    }

    public void extendWobbleGoal(double pow) {
        wge.setPower(pow);
    }

    public void stopWobbleGoal() {
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0);
    }

    public void startOdoThreadAuto(final LinearOpMode op){
        CodeSeg run = new CodeSeg() {
            @Override
            public void run() {
                updateOdometry();
            }
        };
        Stage exit = new Stage() {
            @Override
            public boolean run(double in) {
                return op.isStopRequested();
            }
        };
//        odometryThread = new TerraThread(run, exit);
        odometryThread = new TerraThread(run, exit);
        Thread t = new Thread(odometryThread);
        t.start();
    }

    public void startOdoThreadTele(){
        CodeSeg run = new CodeSeg() {
            @Override
            public void run() {
                updateOdometry();
            }
        };
        odometryThread = new TerraThread(run);
        Thread t = new Thread(odometryThread);
        t.start();

    }

    public void stopOdoThread() {
        if(odometryThread != null) {
            odometryThread.stop();
        }
    }

    public void resetHeadingUsingGyro(){
        odometry.resetHeading(angularPosition.getHeading(odometry.h));
    }

    public void resetPosUsingDisSensors(){
        odometry.resetPos(localizer.getPos(odometry.getPos(), odometry.h));
    }

    public void optimizeOdometry(){
        if(odometryTime.seconds() > (1/Constants.GYRO_UPDATE_RATE)){
            odometryTime.reset();
//            resetHeadingUsingGyro();
            //SKETYCHHHHHHHHHHHHHHHHHHHHH WHYYYYYYYYYYYYYYYYYYYYYYYYYYY????
            optimizeHeading();
        }
    }

    public void initWobbleGoal(){
        switch (wgStartMode){
            case 0:
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setTargetPosition((int) (45*Constants.NEV_DEGREES_TO_TICKS));
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                moveArm(Math.abs(1) * Math.signum(45 - getArmPos()));
                wgStartMode++;
                break;
            case 1:
                controlWGE(1);
                if(!arm.isBusy()){
                    wgStartMode++;
                }
                break;
            case 2:
                moveArm(0);
                wge.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                wgStartMode++;
                break;
            case 3:
                controlWGE(1);
                if(isControlWgeDone(1)) {
                    wgStartMode++;
                    wge.setPower(0);
                }
                break;
            case 4:
                break;
        }

    }

    public void defineShooter(){
        shooter.addStage(rh2, -1);
        shooter.addCustom(new CodeSeg() {
            @Override
            public void run() {
                intaking = false;
                autoAimer.setOuttakePos(getLocalizerPos());
                autoAimer.updateTargetSpeed();
            }
        });
        shooter.addOuttake(outr, outl, 1300, 1600);
//        shooter.addOuttake(outr, outl, autoAimer.getOutlTargetVel() * Constants.GO_RAD_TO_TICKS, autoAimer.getOutrTargetVel() * Constants.GO_RAD_TO_TICKS);
        shooter.addStage(rp, pushControl, 1 , 0.5);
        shooter.addStage(rh2, 0);
        shooter.addCustom(new CodeSeg() {
            @Override
            public void run() {
                fastMode = false;
            }
        });
        shooter.addWait(1);
//        shooter.addPause();
//        shooter.addStage(rp, pushControl, 2, 0.01);
//        shooter.addWait(1);
        for (int i = 0; i < 3; i++) {
            shooter.addStage(rp, pushControl, 2, 0.01);
            shooter.addWait(0.25);
            shooter.addStage(rp, pushControl.getPos(1)-0.03, 0.01);
            shooter.addWait(0.25);
        }
        shooter.addOuttake(outr, outl, 0, 0);
        shooter.addStage(rp, pushControl, 0,  0.01);
//        shooter.addCustom(new CodeSeg() {
//            @Override
//            public void run() {
//                fastMode = true;
//            }
//        });
        shooter.addPause();
        autoModules.add(shooter);
    }
    public void defineAimer(){
        aimer.addAimer(this);
        aimer.addPause();
        autoModules.add(aimer);
    }

    public void defineWobbleGoal(){
        wobbleGoal.addClaw(this, 2);
        wobbleGoal.addControlWGE(this, 1);
        wobbleGoal.addWobbleGoal(this, -10, 1);
        wobbleGoal.toggleFastMode(this);
        wobbleGoal.addPause();
        wobbleGoal.addClaw(this, 0);
        wobbleGoal.addWait(1);
        wobbleGoal.toggleFastMode(this);
        wobbleGoal.addWobbleGoal(this, 120, 1);
        wobbleGoal.addControlWGE(this, 0.5);
        wobbleGoal.holdWobbleGoalAndPause(this);
        wobbleGoal.addMove(this, new double[]{0,20,0}, true);
        wobbleGoal.addClaw(this, 1);
        wobbleGoal.addWobbleGoal(this, 160, 1);
        wobbleGoal.addClaw(this, 2);
        wobbleGoal.addWait(0.7);
        wobbleGoal.addWobbleGoal(this, 45, 1);
        wobbleGoal.addPause();

        autoModules.add(wobbleGoal);
    }



    public void definePowershot(){
        powerShot.addCustom(new CodeSeg() {
            @Override
            public void run() {
                updateLocalizer();
                updateOdoWithSensors();
            }
        });
        powerShot.addStage(rh2, -1);
        powerShot.addOuttake(outr, outl, 1300, 1600);
        powerShot.addStage(rp, pushControl, 1 , 0.5);
        powerShot.addStage(rh2, 0);
        powerShot.toggleFastMode(this);
        powerShot.addMoveGlobal(this, new double[]{126, 176, 0});
        powerShot.addWait(1);
        for (int i = 0; i < 3; i++) {
            powerShot.addStage(rp, pushControl, 2, 0.01);
            powerShot.addWait(0.3);
            powerShot.addStage(rp, pushControl.getPos(1) - 0.03, 0.01);
            powerShot.addWait(0.3);
            if(i < 2) {
//                powerShot.addMove(this, new double[]{0, -1, -10}, false);
                powerShot.addMove(this, new double[]{20, 0, 0}, false);
            }
        }
        powerShot.addOuttake(outr, outl, 0, 0);
        powerShot.addStage(rp, pushControl, 0,  0.01);
//        powerShot.toggleFastMode(this);
        powerShot.addPause();
        autoModules.add(powerShot);
    }

}
