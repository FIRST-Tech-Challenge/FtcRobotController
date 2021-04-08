package global;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    public DcMotor outr;
    public DcMotor outl;
    public DcMotor arm;

    public DcMotor in;

    public CRServo rh;
    public CRServo rh2;

    public CRServo wge;
    public Rev2mDistanceSensor wgp;

    public Servo cll;
    public Servo clr;
    public Servo rp;


    public Cycle pushControl = new Cycle(0.1, 0.25, 0.32);

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

    public boolean fastMode = true;
    public boolean wgeStartMode = true;

    public AutoModule shooter = new AutoModule(); // 0

    public ArrayList<AutoModule> autoModules = new ArrayList<>();

    public ButtonController outtakeButtonController = new ButtonController();

    public Odometry odometry = new Odometry();

    public TerraThread odometryThread;



    public void init(HardwareMap hwMap) {

        l1 = getMotor(hwMap, "l1", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2 = getMotor(hwMap, "l2", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1 = getMotor(hwMap, "r1", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2 = getMotor(hwMap, "r2", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        in = getMotor(hwMap, "in", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outr = getMotor(hwMap, "outr", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outl = getMotor(hwMap, "outl", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm = getMotor(hwMap, "arm", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        outr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rh = getCRServo(hwMap, "rh", CRServo.Direction.FORWARD);
        rh2 = getCRServo(hwMap, "rh2", CRServo.Direction.REVERSE);
        cll = getServo(hwMap, "cll", Servo.Direction.FORWARD, Constants.CLL_GRAB);
        clr = getServo(hwMap, "clr", Servo.Direction.REVERSE, Constants.CLL_OPEN);
        rp = getServo(hwMap, "rp", Servo.Direction.FORWARD, Constants.RP_START);
        wge = getCRServo(hwMap, "wge", CRServo.Direction.REVERSE);

        wgp = hwMap.get(Rev2mDistanceSensor.class, "wgp");

        angularPosition.init(hwMap);
        localizer.init(hwMap);

        shooter.addStage(rh, -1);
        shooter.addStage(rh2, -1);
        shooter.addStage(rp, pushControl, 1 , 0.5);
        shooter.addStage(rh2, 0);
        shooter.addStage(rh, 1);
        shooter.addWait(0.3);
        shooter.addStage(rh, 0);
        shooter.addStage(0.8, outl);
        shooter.addStage( 0.4, outr);
        shooter.addPause();
        for(int i = 0; i < 3; i++) {
            shooter.addStage(rh, -1);
            shooter.addStage(rp, pushControl, 2, 0.3);
            shooter.addStage(rh, 0);
            shooter.addStage(rp, pushControl, 1, 0.3);
        }
        shooter.addStage(rp, pushControl, 0, 0.3);
        autoModules.add(shooter);

        odometry.updateEncoderPositions(getLeftOdo(), getCenterOdo(), getRightOdo());

        limits.addLimit(arm, Constants.WG_LOWER_LIMIT, Constants.WG_UPPER_LIMIT);
        limits.addLimit(wge, 0, Constants.WGE_UPPER_LIMIT);

    }

    public DcMotor getMotor(HardwareMap hwMap, String name, DcMotor.Direction dir, DcMotor.ZeroPowerBehavior zpb, DcMotor.RunMode mode){
        DcMotor dcMotor = hwMap.get(DcMotor.class, name);
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

    public void moveTeleOp(double f, double s, double t){
        double restForwardPow = 0.05;
        double restStrafePow = 0.1;
        double restTurnPow = 0.1;

        if(fastMode) {
            move((f*(1-restForwardPow)) + Math.signum(f)*restForwardPow, (s*(1-restStrafePow))+Math.signum(s)*restStrafePow, 0.9*t*(1-restTurnPow)+ (Math.signum(t) * restTurnPow));
        }else{
            move((f*0.2) + Math.signum(f)*restForwardPow, (s*0.2)+Math.signum(s)*restStrafePow, (t*0.2)+Math.signum(t)*restTurnPow);
        }

    }

    public void moveArm(double p){
        if (isArmInLimits(p)) {
            arm.setPower(p + getRestPowArm());
            if(isWgeInLimits(p)) {
                updateWge();
            }else{
                wge.setPower(0);
            }
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
            double targetPos = Constants.WGE_UPPER_LIMIT - Constants.WGE_UPPER_LIMIT / (1 + Math.pow(Math.E, -c * (armPos - 0.5 * Constants.WG_UPPER_LIMIT - Constants.WG_LOWER_LIMIT)));
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

    public boolean areAutomodulesRunning(){
        for (AutoModule a: autoModules) {
            if (a.isExecuting()) { return true; }
        }
        return false;
    }

    public void stopAllAutomodules(){
        for (AutoModule a: autoModules) {
            a.stop();
        }
    }

    public void toggleOuttake(boolean in) {
        if (outtakeButtonController.isPressing(in)) {
            outtaking = !outtaking;
            resetOuttake();
        }
    }

    public void resetOuttake() {
        autoAimer.update(odometry.getPos());
        autoAimer.resetOuttake(getLeftAngPos(), getRightAngPos());
    }

    public double getRightAngPos(){
        return (outr.getCurrentPosition()/Constants.GOBUILDA1_Ticks)*Constants.pi2;
    }

    public double getLeftAngPos(){
        return (outl.getCurrentPosition()/Constants.GOBUILDA1_Ticks)*Constants.pi2;
    }



    public void outtakeWithCalculations() {
        if (outtaking) {
            autoAimer.update(odometry.getPos());
            outr.setPower(autoAimer.getOutrPow(getRightAngPos()));
            outl.setPower(autoAimer.getOutlPow(getLeftAngPos()));
            //rh.setPower(-0.5);
        } else {
            outr.setPower(0);
            outl.setPower(0);
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

    public void extendWobbleGoal(double pow) {
        wge.setPower(pow);
    }

    public void startWobbleGoalWithEncoders(double pos, double pow) {
        pos *= Constants.NEV_DEGREES_TO_TICKS;
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition((int) pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(pow);
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
                return !op.opModeIsActive();
            }
        };
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



}
