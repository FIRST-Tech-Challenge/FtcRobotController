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
import java.util.Arrays;

import globalfunctions.Constants;
import globalfunctions.Optimizer;
import globalfunctions.Storage;
import globalfunctions.TerraThread;
import telefunctions.AutoModule;
import telefunctions.ButtonController;
import telefunctions.Cycle;
import telefunctions.Limits;
import util.Stage;
import util.CodeSeg;

public class TerraBot {

    //Drive train Motors
    public DcMotor r1;
    public DcMotor l1;
    public DcMotor r2;
    public DcMotor l2;

    //Outtake Motors
    public DcMotorEx outr;
    public DcMotorEx outl;

    //Wobble goal motor
    public DcMotor arm;

    //Intake motor
    public DcMotor in;

    //Ring Shooter crservo
    public CRServo rs;
    //Ring Knocker crservo
    public CRServo rk;
    //Wobble goal extender crservo
    public CRServo wge;
    //Wobble goal pos distance sensor
    public Rev2mDistanceSensor wgp;

    //Wobble goal claw servos
    public Servo cll;
    public Servo clr;

    //Wobble goal claw controls
    public Cycle cllControl = new Cycle(0.2, 0.5, 1);
    public Cycle clrControl = new Cycle(1, 0.5, 0.0);

    // Ring knock-down servos
    public Servo fls;
    public Servo frs;

    // Ring knock-down servos controls
    public Cycle flsControl = new Cycle(Constants.FLS_CLOSED, 0.5);
    public Cycle frsControl = new Cycle(Constants.FRS_CLOSED, 0.5);

    //AutoAimer - Has methods for angle to target and shooting
    public AutoAimer autoAimer = new AutoAimer();
    //Angular Position - Has methods for gyro sensors
    public AngularPosition angularPosition = new AngularPosition();
    //Localizer - Has methods for distance sensors
    public Localizer localizer = new Localizer();
    //Limits - stops motors in range
    public Limits limits = new Limits();

    //Is the robot intaking?
    public boolean intaking = false;
    //Is the robot outtaking?
    public boolean outtaking = false;


    //Is the robot in fastMode?
    public boolean fastMode = false;
    //Has the wobble goal not been initialized yet?
    public boolean wgeStartMode = true;
    //What stage is the initalization of the wobble goal at?
    public int wgStartMode = 0;
    //Start Position of wobble goal in degs
    public double wgStart = 0;


    //Automodule for shooting
    public AutoModule shooter = new AutoModule(); // 0

    //Automodule for wobble goal
    public AutoModule wobbleGoal = new AutoModule();

    //Automodule for powershot
    public AutoModule powerShot = new AutoModule();

    //List of AutoModules
    public ArrayList<AutoModule> autoModules = new ArrayList<>();

    //Button Controls - Make delay between clicks to only count them once
    public ButtonController outtakeButtonController = new ButtonController();
    public ButtonController fastModeController = new ButtonController();
    public ButtonController powerShotController = new ButtonController();

    //Odometry for position of robot
    //TODO
    // Make this more accurate pls?
    public Odometry odometry = new Odometry();

    //Thread for odometry at 100 htz
    public TerraThread odometryThread;
    //odometry timer - used for optimizing odometry using gyro
    public ElapsedTime odometryTime = new ElapsedTime();
    //Game time - used for telling the game time
    public ElapsedTime gameTime = new ElapsedTime();


    //Is movement available in teleop?
    public boolean isMovementAvailable = true;

    //Is in powershot mode?
    public boolean powershotMode = false;

    //Storage - Used to save files
    public Storage storage = new Storage();




    public void init(HardwareMap hwMap) {
        //Get drivetrain
        l1 = getMotor(hwMap, "l1", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2 = getMotor(hwMap, "l2", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1 = getMotor(hwMap, "r1", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2 = getMotor(hwMap, "r2", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Get Intake, Outtake, Wobble Goal
        in = getMotor(hwMap, "in", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outr = getMotorEx(hwMap, "outr", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outl = getMotorEx(hwMap, "outl", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm = getMotor(hwMap, "arm", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Reset encoders for outtake and wobble goal
        outr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Get CR Servos - Ring Shooter, Ring knocker, Wobble goal extender
        rs = getCRServo(hwMap, "rs", CRServo.Direction.FORWARD);
        rk = getCRServo(hwMap, "rk", CRServo.Direction.FORWARD);
        wge = getCRServo(hwMap, "wge", CRServo.Direction.REVERSE);

        //Get claw Servos
        cll = getServo(hwMap, "cll", Servo.Direction.FORWARD, Constants.CLL_GRAB);
        clr = getServo(hwMap, "clr", Servo.Direction.REVERSE, Constants.CLL_OPEN);

        // Get ring knock-down servos
        fls = getServo(hwMap, "fls", Servo.Direction.FORWARD, Constants.FLS_CLOSED);
        frs = getServo(hwMap, "frs", Servo.Direction.REVERSE, Constants.FRS_CLOSED);

        //Get wobble goal pos distance sensor
        wgp = hwMap.get(Rev2mDistanceSensor.class, "wgp");

        //Initialize angular position and localizer
        angularPosition.init(hwMap);
        localizer.init(hwMap);

        //Update odometry positions
        odometry.updateEncoderPositions(getLeftOdo(), getCenterOdo(), getRightOdo());

        //Add limits for wobble goal arm and extender
        limits.addLimit(arm, Constants.WG_LOWER_LIMIT, Constants.WG_UPPER_LIMIT);
        limits.addLimit(wge, 0, Constants.WGE_UPPER_LIMIT);

        //Set coeffs for shooter
        outr.setVelocityPIDFCoefficients(54, 0, 0, 14);
        outl.setVelocityPIDFCoefficients(54, 0, 0, 14);

        //Define Automodules
        defineShooter();
        defineWobbleGoal();
        definePowershot();
    }

    //Helper methods for getting hardware
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

    //Method for moving
    public void move(double f, double s, double t){
        l1.setPower(f+s-t);
        l2.setPower(-f+s+t);
        r1.setPower(f-s+t);
        r2.setPower(-f-s-t);
    }
    //Intake power p and check if not outtake to control ring shooter
    public void intake(double p){
        in.setPower(p);
        if (!outtaking) { shootRings(-Math.signum(p)*Constants.RS_POW);}
    }
    //Outtake at power p
    public void outtake(double p){
        outr.setPower(p);
        outl.setPower(p);
    }
    //Claw to posleft and posright
    public void claw(double posLeft, double posRight){
        cll.setPosition(posLeft);
        clr.setPosition(posRight);
    }
    //Set claw to controller positions based on index
    public void setClawPos(int ind) {
        cllControl.changeCurr(ind);
        clrControl.changeCurr(ind);
        claw(cllControl.getPos(ind), clrControl.getPos(ind));
    }
    //Update claw for teleop
    public void updateClaw(boolean dpl, boolean dpr) {
        if (dpr) {
            claw(cllControl.getPos(0), clrControl.getPos(0));
        } else if (dpl) {
            claw(cllControl.getPos(2), clrControl.getPos(2));
        }
    }
    //Extend WGE at pow p
    public void extendWobbleGoal(double pow) {
        wge.setPower(pow);
    }
    public void shootRings(double pow){
        rs.setPower(pow);
    }

    //Update intake for teleop
    public void updateIntake(boolean left_bumper, boolean right_bumper) {
        //Set intake on with right bumper and turn it off with left bumper and also go backward with left bumper
        if(right_bumper){
            intaking = true;
        }else if(left_bumper){
            intaking = false;
            intake(-0.5);
        }else if(intaking){
            intake(1);
        }else{
            intake(0);
        }
    }

    //Move for teleop
    public void moveTeleOp(double f, double s, double t, double rt, double lt){
        //If movement is available in teleop then move
        if(isMovementAvailable){
            //Fastmode movements are about twice as fast as not fastmode
            if (fastMode) {
                move(Math.signum(f) * Math.pow(Math.abs(f), 0.5), Math.signum(s) * Math.pow(Math.abs(s), 0.5), Math.signum(t) * Math.pow(Math.abs(t), 0.5));
            } else {
                move(0.5 * Math.signum(f) * Math.pow(Math.abs(f), 0.5), 0.5 * Math.signum(s) * Math.pow(Math.abs(s), 0.5), 0.4 * Math.signum(t) * Math.pow(Math.abs(t), 0.5));
            }
        }
        //Switch fastmode using button controller
        if (fastModeController.isPressedOnce(rt > 0)) {
            fastMode = !fastMode;
            isMovementAvailable = true;
        }

        if(powerShotController.isPressedOnce(lt > 0)){
            powershotMode = !powershotMode;
        }
    }
    //Move wobble goal arm at pow p with restpow
    public void moveArm(double p){
        if (isArmInLimits(p)) {
            arm.setPower(p + getRestPowArm());
        }
    }
    //Get rest pow for arm
    public double getRestPowArm(){
        return Constants.WG_REST_POW*Math.cos(Math.toRadians(getArmPos()));
    }

    //Move wobble goal arm with encoder and without wobble goal extender to pos deg and at pow p
    public void moveArmWithEncWithoutWGE(double deg, double pow){
        //Set arm to use encoder
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition((int) ((deg - wgStart)*Constants.NEV_DEGREES_TO_TICKS));
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Move arm at pow based on pos
        moveArm(Math.abs(pow) * Math.signum(deg - getArmPos()));
        while (arm.isBusy()){}
        moveArm(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Get arm pos in degs using wgStart pos
    public double getArmPos(){
        return ((arm.getCurrentPosition()/Constants.NEV_DEGREES_TO_TICKS) + wgStart);
    }
    //Use distance sensor to get distance of wge
    public double getWgePos() { return wgp.getDistance(DistanceUnit.CM) - Constants.WGE_START; }
    //Check if the wobble goal arm is in limits
    public boolean isArmInLimits(double dir){
        return limits.isInLimits(arm, dir, getArmPos());
    }
    //Check if the wobble goal extender is in limits
    public boolean isWgeInLimits(double dir) { return limits.isInLimits(wge, dir, getWgePos()) && !Optimizer.inRange(getArmPos(), Constants.WGE_IGNORE_RANGE); }

    //Control wobble goal extender to a certain pos
    public void controlWGE(double pos){
        //If its not in start mode then move it
        if (!wgeStartMode) {
            //Move it until it reaches within a certain accuracy
            double wgePos = getWgePos();
            double targetPos =  Constants.WGE_UPPER_LIMIT*pos;
            if (Math.abs(targetPos - wgePos) < Constants.WGE_ACC) {
                wge.setPower(0);
            } else {
                wge.setPower(Math.signum(targetPos - wgePos));
            }
        } else {
            //Set startmode to true when arm pos is greater than 40 degs
            wgeStartMode = getArmPos() < 40;
        }
    }
    //Is controlling the wobble goal extender done?
    public boolean isControlWgeDone(double pos){
        double targetPos =  Constants.WGE_UPPER_LIMIT*pos;
        return Math.abs(targetPos - getWgePos()) < Constants.WGE_ACC;
    }
    //Are any automodules running checks if they are inited
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
    //Stops all automodules
    public void stopAllAutomodules(){
        for (AutoModule a: autoModules) {
            if(a.inited) {
                a.stop();
            }
        }
    }
    //Get angular position of outtake motors
    public double getRightAngPos() {return (outr.getCurrentPosition() / Constants.GOBUILDA1_Ticks) * Constants.pi2; }
    public double getLeftAngPos(){ return (outl.getCurrentPosition()/Constants.GOBUILDA1_Ticks)*Constants.pi2; }

    //Get angular velocities of outtake motors
    public double getRightAngVel(){ return (outr.getVelocity()/Constants.GOBUILDA1_Ticks)*Constants.pi2;}
    public double getLeftAngVel(){ return (outl.getVelocity()/Constants.GOBUILDA1_Ticks)*Constants.pi2; }

    //Get position of the robot using localizer
    public double[] getLocalizerPos() {
        return localizer.getPos();
    }

    //Updates localizer with gyro heading
    public void updateLocalizerWithHeading() {
        localizer.update(angularPosition.getHeadingGY());
    }
    //Sets heading of the robot by updating localizer, odometry, and angular position
    public void setHeading(double angle) {
        localizer.update(angle);
        odometry.resetHeading(angle);
        angularPosition.resetGyro(angle);
    }
    //Update odometry with localizer pos
    public void updateOdoWithLocalizer() {
        updateLocalizerWithHeading();
        odometry.resetPos(getLocalizerPos());
    }
    //Update odometry with localizer and check
    public void updateOdoWithLocalizerAndCheck(){
        updateLocalizerWithHeading();
        odometry.resetPos(localizer.getPos(odometry.getPos()));
    }
    //Update odometry with gyro heading
    public void updateOdoWithGyro(){
        odometry.resetHeading(angularPosition.getHeadingGY());
    }
    //Update odometry with gyro heading and check if odometry heading is already close
    public void updateOdoWithGyroAndCheck(){
        odometry.resetHeading(angularPosition.getHeading(odometry.h));
    }

    //Outtake with autoAimer calculations
    public void outtakeWithCalculations() {
        if(outtaking){
            if(outr.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
                outr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                outl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                autoAimer.setOuttakePos(odometry.getPos());
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
            if (!intaking) { shootRings(0); }
        }
    }
    //Get odometry positions in ticks
    public int getLeftOdo() { return -in.getCurrentPosition(); }
    public int getRightOdo() { return l2.getCurrentPosition(); }
    public int getCenterOdo() {return r2.getCurrentPosition(); }

    //Update Odometry positions
    public void updateOdometry() {
        odometry.updateGlobalPosition(getLeftOdo(), getCenterOdo(), getRightOdo());

    }

    //Optime odometry Heading to [-180,180] range
    public void optimizeOdometryHeading(){ odometry.resetHeading(Optimizer.optimizeHeading(odometry.h)); }


    //Start odometry thread for autonomous
    public void startOdoThreadAuto(final LinearOpMode op){
        CodeSeg run = () -> {
            updateOdometry();
            updateOdometryUsingSensors();
        };
        Stage exit = new Stage() {
            @Override
            public boolean run(double in) {
                return op.isStopRequested();
            }
        };
        odometryThread = new TerraThread(run, exit);
        Thread t = new Thread(odometryThread);
        t.start();
    }

    //Start odometry thread for teleop
    public void startOdoThreadTele(){
        CodeSeg run = () -> {
            updateOdometry();
            updateOdometryUsingSensors();
        };
        odometryThread = new TerraThread(run);
        Thread t = new Thread(odometryThread);
        t.start();
    }

    //Stop odometry thread
    public void stopOdoThread() {
        if(odometryThread != null) {
            odometryThread.stop();
        }
    }

    public void updateOdometryUsingSensors(){
        if(odometryTime.seconds() > (1/Constants.UPDATE_ODOMETRY_WITH_SENSORS_RATE)){
            odometryTime.reset();
            updateOdoWithGyroAndCheck();
            updateOdoWithLocalizerAndCheck();
        }
    }

    //Get Angle to Goal
    public double getRobotToGoalAngle(){
        return autoAimer.getRobotToGoalAngle(odometry.getPos());
    }

    //Initialize Wobble goal
    public void initWobbleGoal(){
        switch (wgStartMode){
            case 0:
                //Move arm to 45 degs
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setTargetPosition((int) ((45 - wgStart)*Constants.NEV_DEGREES_TO_TICKS));
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                moveArm(Math.abs(1) * Math.signum(45 - getArmPos()));
                wgStartMode++;
                break;
            case 1:
                //Move wge to all the way out and stop moving arm
                controlWGE(1);
                if(!arm.isBusy()){
                    wgStartMode++;
                }
                break;
            case 2:
                //Stop moving arm and wobble goal extender
                moveArm(0);
                wge.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                wgStartMode++;
                break;
            case 3:
                //Move wge to right pos
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

    //TODO
    // Define shooter and powershot
    public void defineShooter(){
        shooter.init(this);
        shooter.addCustom(() -> {
            outtaking = true;
            autoAimer.shotMode = 0;
        });
        shooter.addWait(0.1);
        shooter.addTurnToGoal();
        shooter.addWait(0.2);
        shooter.addStage(rs, Constants.RS_POW);
        shooter.addWait(1);
        shooter.addStage(0, outr, outl);
        shooter.addStage(rs, 0);
        shooter.addCustom(() -> outtaking = false);
        shooter.addPause();
        autoModules.add(shooter);
    }

    public void definePowershot(){
        powerShot.init(this);
        powerShot.addCustom(() -> outtaking = true);

        for (int i = 1; i < 4; i++) {
            powerShot.changeAutoAimerMode(i);
            powerShot.addWait(0.2);
            powerShot.addTurnToGoal();
            powerShot.addStage(rs, Constants.RS_POW);
            powerShot.addWait(0.2);
            powerShot.addStage(rs, 0);
        }
        powerShot.addStage(0, outr, outl);
        powerShot.addStage(rs, 0);
        powerShot.addCustom(() -> outtaking = false);
        powerShot.addPause();
        autoModules.add(powerShot);
    }

    //Define wobble goal automodule
    public void defineWobbleGoal(){
        wobbleGoal.init(this);
        wobbleGoal.addClaw( 2); //Open claw
        wobbleGoal.addControlWGE(1); //Mode wge out
        wobbleGoal.addWobbleGoal(-10, 1); //Move wg arm down
        wobbleGoal.addCustom(() -> fastMode = true); //Set to slowmode
        wobbleGoal.addPause(); // Wait for driver
        wobbleGoal.addClaw( 0); //Close claw
        wobbleGoal.addWait(0.5); // Wait 0.5
        wobbleGoal.addWobbleGoal( 120, 1); //Move wg arm up
        wobbleGoal.addControlWGE( 0.5); //move wge in to halfway
        wobbleGoal.holdWobbleGoalAndPause(); //Hold wobble gaol arm pause
        wobbleGoal.addMove(new double[]{0,20,0}, true); //Move forward 20 cm
        wobbleGoal.addClaw( 1); //open claw halfway
        wobbleGoal.addWobbleGoal( 160, 1); //Move woggle goal arm down
        wobbleGoal.addClaw( 2); //open claw
        wobbleGoal.addWait(0.7); //wait to drop
        wobbleGoal.addWobbleGoal( 45, 1); //Move arm forward
        wobbleGoal.addPause(); //Pause for next time
        autoModules.add(wobbleGoal);
    }


    //Save data for telop in auton
    public void saveForTele() {
        storage.makeOutputFile("save");
        storage.saveText(Double.toString(getArmPos()), "wgPos");
        storage.saveText(Double.toString(angularPosition.getHeadingGY()), "heading");
        storage.saveText(Arrays.toString(odometry.getPos()), "pos");
    }
    //Read date from auton in telop
    public void readFromAuton() {
        storage.makeOutputFile("save");
        wgStart = Double.parseDouble(storage.readText("wgPos"));
        angularPosition.resetGyro(Double.parseDouble(storage.readText("heading")));
        odometry.resetPos(Optimizer.fromString(storage.readText("pos")));
        odometry.resetHeading(angularPosition.getHeadingGY());
        updateOdoWithLocalizerAndCheck();
    }




}
