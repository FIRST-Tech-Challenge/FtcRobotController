package global;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.revextensions2.ExpansionHubEx;

import autofunctions.Odometry;
import autofunctions.Path;
import telefunctions.AutoModule;
import telefunctions.Cycle;
import telefunctions.Limits;
import telefunctions.ServoController;
import telefunctions.SpeedController;
import util.CodeSeg;
import util.ThreadHandler;


public class TerraBot {

    public DcMotor l1;
    public DcMotor l2;
    public DcMotor r1;
    public DcMotor r2;

    public DcMotor in;
    public DcMotor outr;
    public DcMotor outl;
    public DcMotor arm;

    public Servo slr;
    public Servo sll;
    public Servo ssr;
    public Servo ssl;
    public Servo st;
    public Servo sgr;
    public Servo sgl;

    public BNO055IMU gyro;

    public boolean intaking = false;
    public boolean outtaking = false;
    public boolean fastmode = true;
    public boolean powershot = false;

    public int resettingArm = 0;

    public double turnStart = 0.25;
    public double grabStart = 0.7;
    public double liftStart = 0.0;
    public double liftSecond = 0.45;
    public double shootStartR = 0.06;
    public double shootStartL = 0.05;
    public double intakeSpeed = 1;
    public double outtakeSpeed = 0.37;
    public double powerShotSpeed = 0.3;
    public double maxArmPos = 215;
    public double heading = 0;
    public double lastAngle = 0;
    public double lastArmAngle = 0;

    public final double NEVEREST256_TICKS = 7168;
    public final double NEV_DEGREES_TO_TICKS = NEVEREST256_TICKS/360;
    public final double GOBUILDA1_Ticks = 28;
    public final double GO_DEGREES_TO_TICKS = GOBUILDA1_Ticks/360;
    public final double MAX_OUTTAKE_SPEED = 32400;

    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime timer2 = new ElapsedTime();
    public ElapsedTime gameTime = new ElapsedTime();

    public Cycle grabControl = new Cycle(grabStart, 0.45);
    public Cycle liftControl = new Cycle(liftStart, liftSecond);
    public Cycle shootControlR = new Cycle(0.0, shootStartR, 0.22, 0.25);
    public Cycle shootControlL = new Cycle(0.0, shootStartL, 0.13, 0.26);

    public ServoController turnControl = new ServoController(turnStart, 0.0, 0.7);

    public AutoModule shooter = new AutoModule();
    public AutoModule powerShot = new AutoModule();
    public AutoModule wobbleGoal = new AutoModule();
    public AutoModule wobbleGoal2 = new AutoModule();
    public AutoModule goback = new AutoModule();

    public Limits limits = new Limits();

    //d = 0.00024
    public SpeedController outrController = new SpeedController(0.2, 0.0, 0.0, outtakeSpeed);
    public SpeedController outlController = new SpeedController(0.2, 0.0, 0.0, outtakeSpeed);

    public Odometry odometry = new Odometry();

    public ThreadHandler threadHandler = new ThreadHandler();

    public ExpansionHubEx expansionHub;







    public void init(HardwareMap hwMap) {

        l1 = hwMap.get(DcMotor.class, "l1");
        l2 = hwMap.get(DcMotor.class, "l2");
        r1 = hwMap.get(DcMotor.class, "r1");
        r2 = hwMap.get(DcMotor.class, "r2");
        in = hwMap.get(DcMotor.class, "in");
        outr = hwMap.get(DcMotor.class, "outr");
        outl = hwMap.get(DcMotor.class, "outl");
        arm = hwMap.get(DcMotor.class, "arm");

        slr = hwMap.get(Servo.class, "slr");
        sll = hwMap.get(Servo.class, "sll");
        ssr = hwMap.get(Servo.class, "ssr");
        ssl = hwMap.get(Servo.class, "ssl");
        st = hwMap.get(Servo.class, "st");
        sgr = hwMap.get(Servo.class, "sgr");
        sgl = hwMap.get(Servo.class, "sgl");

        gyro = hwMap.get(BNO055IMU.class, "gyro");

        expansionHub = hwMap.get(ExpansionHubEx.class, "Expansion Hub 2");



        




        l1.setPower(0);
        l2.setPower(0);
        r1.setPower(0);
        r2.setPower(0);
        in.setPower(0);
        outr.setPower(0);
        outl.setPower(0);
        arm.setPower(0);


        slr.setPosition(liftStart);
        sll.setPosition(1-liftStart);
        ssr.setPosition(shootStartR);
        ssl.setPosition(1-shootStartL);
        st.setPosition(turnStart);
        sgr.setPosition(grabStart);
        sgl.setPosition(1-grabStart);


        l1.setDirection(DcMotorSimple.Direction.REVERSE);
        l2.setDirection(DcMotorSimple.Direction.FORWARD);
        r1.setDirection(DcMotorSimple.Direction.FORWARD);
        r2.setDirection(DcMotorSimple.Direction.REVERSE);
        in.setDirection(DcMotorSimple.Direction.REVERSE);
        outl.setDirection(DcMotorSimple.Direction.FORWARD);
        outr.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        slr.setDirection(Servo.Direction.FORWARD);
        sll.setDirection(Servo.Direction.REVERSE);
        ssr.setDirection(Servo.Direction.FORWARD);
        ssl.setDirection(Servo.Direction.REVERSE);
        st.setDirection(Servo.Direction.FORWARD);
        sgr.setDirection(Servo.Direction.FORWARD);
        sgl.setDirection(Servo.Direction.REVERSE);

        l1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        in.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shootControlR.changeCurr(1);
        shootControlL.changeCurr(1);

        resetEncoders();

        defineShooter();
        definePowerShot();
        defineWobbleGoal();
        defineWobbleGoal2();
        defineGoback();


        limits.addLimit(arm, 0, maxArmPos);

        initGyro();

        odometry.init(getLeftOdo(), getMiddleOdo(), getRightOdo());

        resetGyro();




    }

    public void move(double f, double s, double t){
        l1.setPower(f-s-t);
        l2.setPower(-f-s+t);
        r1.setPower(f+s+t);
        r2.setPower(-f+s-t);
    }

    public void moveTeleOp(double f, double s, double t){
        if(fastmode) {
            move(f, s, t);
        }else{
            move((f*0.2) + Math.signum(f)*0.05, (s*0.2)+Math.signum(s)*0.1, (t*0.2)+Math.signum(t)*0.3);
        }
    }


    public void intake(double p){
        in.setPower(p);
    }

    public void outtake(double p){
        outr.setPower(p*getVoltageScale());
        outl.setPower(p*getVoltageScale());
    }

    public void turnArm(double p){
        arm.setPower(p);
    }

    public void lift(double pos){
        slr.setPosition(pos);
        sll.setPosition(pos);
    }

    public void shoot(double pr, double pl){
        ssr.setPosition(pr);
        ssl.setPosition(pl);
    }

    public void turnWobbleArm(double pos){
        st.setPosition(pos);
    }

    public void turnArmWithEnc(double deg, double pow){
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(degreesToTicks(deg));
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(pow);
        while (arm.isBusy()){}
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void grab(double pos){
        sgr.setPosition(pos);
        sgl.setPosition(pos);
    }

    public void defineShooter(){

        shooter.addStage(in, 1.0, 0.01);
        shooter.addCustom(new CodeSeg() {
            @Override
            public void run() {
                fastmode = false;
            }
        }, 0.01);
        shooter.addStage(slr, liftControl.getPos(1), 0.01);
        shooter.addStage(sll, liftControl.getPos(1), 1.5);
        shooter.addStage(ssr, shootControlR.getPos(2), 0.01);
        shooter.addStage(ssl, shootControlL.getPos(2), 1);
        shooter.addStage(in, 0.0, 0.01);
        shooter.addCustom(new CodeSeg() {
            @Override
            public void run() {
               intaking = false;
            }
        }, 0.01);
        shooter.addWaitUntil();
        for(int i = 0; i < 3;i++) {
            shooter.addStage(ssr, shootControlR.getPos(3), 0.01);
            shooter.addStage(ssl, shootControlL.getPos(3), 0.5);
            shooter.addStage(ssr, shootControlR.getPos(2), 0.01);
            shooter.addStage(ssl, shootControlL.getPos(2), 0.5);
        }
        shooter.addCustom(new CodeSeg() {
            @Override
            public void run() {
                fastmode = true;
            }
        }, 0.01);
        shooter.addDelay(1);
    }

    public void definePowerShot(){
//        powerShot.addCustomOnce(new CodeSeg() {
//            @Override
//            public void run() {
//                startOdoThreadTele();
//            }
//        });
        powerShot.addStage(in, 1.0, 0.01);
        powerShot.addCustom(new CodeSeg() {
            @Override
            public void run() {
                fastmode = false;
            }
        }, 0.01);
        powerShot.addStage(slr, liftControl.getPos(1), 0.01);
        powerShot.addStage(sll, liftControl.getPos(1), 1.5);
        powerShot.addStage(ssr, shootControlR.getPos(2), 0.01);
        powerShot.addStage(ssl, shootControlL.getPos(2), 0.01);
        powerShot.addStage(in, 0.0, 0.01);
        powerShot.addCustom(new CodeSeg() {
            @Override
            public void run() {
                intaking = false;
            }
        }, 0.01);
        powerShot.addWaitUntil();
        powerShot.addCustomOnce(new CodeSeg() {
            @Override
            public void run() { resetAll();}
        });
        for(int i = 0; i < 3;i++) {
            powerShot.addStage(ssr, shootControlR.getPos(3), 0.01);
            powerShot.addStage(ssl, shootControlL.getPos(3), 0.3);
            powerShot.addStage(ssr, shootControlR.getPos(2), 0.01);
            powerShot.addStage(ssl, shootControlL.getPos(2), 0.3);
            if(i < 2) {
                Path path = new Path(i * 17, 0, 0);
                path.addSetpoint(17, 0, 0);
                powerShot.addPath(path, this);
            }
        }
        powerShot.addCustom(new CodeSeg() {
            @Override
            public void run() {
                fastmode = true;
            }
        }, 0.01);
//        powerShot.addCustomOnce(new CodeSeg() {
//            @Override
//            public void run() {
//                stopOdoThreadTele();
//            }
//        });
        powerShot.addDelay(1);
    }
    public void defineWobbleGoal(){
//        wobbleGoal.addStage(ssr, shootControlR.getPos(0), 0.01);
//        wobbleGoal.addStage(ssl, shootControlL.getPos(0), 0.01);
        wobbleGoal.addStage(st, 0.65, 0.1);
//        wobbleGoal.addStage(arm,  1, degreesToTicks(205));
//        wobbleGoal.addWaitUntil();
//        wobbleGoal.addStage(sgl, grabControl.getPos(1), 0.01);
//        wobbleGoal.addStage(sgr, grabControl.getPos(1), 0.5);
//        wobbleGoal.addStage(st, 0.18, 0.01);
//        wobbleGoal.addStage(arm,  1, degreesToTicks(215));
//        wobbleGoal.addStage(slr, liftControl.getPos(1), 0.01);
//        wobbleGoal.addStage(sll, liftControl.getPos(1), 0.5);
////        wobbleGoal.addStage(slr, liftControl.getPos(2), 0.01);
////        wobbleGoal.addStage(sll, liftControl.getPos(2), 0.5);
//        wobbleGoal.addDelay(0.25);
//        wobbleGoal.addStage(st, 0.65, 0.05);
//        wobbleGoal.addStage(slr, liftControl.getPos(0), 0.01);
//        wobbleGoal.addStage(sll, liftControl.getPos(0), 0.01);
//        wobbleGoal.addDelay(0.7);
//        wobbleGoal.addStage(arm, 1, degreesToTicks(210));
//        wobbleGoal.addSave(grabControl, 0);
//        wobbleGoal.addSave(turnControl, 0.65);

    }
    public void defineWobbleGoal2(){
        wobbleGoal2.addStage(st, 0.65, 0.1);
        wobbleGoal2.addStage(arm,  1, degreesToTicks(185));
        wobbleGoal2.addCustomOnce(new CodeSeg() {
            @Override
            public void run() {
                fastmode = false;
            }
        });
        wobbleGoal2.addWaitUntil();
        wobbleGoal2.addStage(sgl, grabControl.getPos(1), 0.01);
        wobbleGoal2.addStage(sgr, grabControl.getPos(1), 0.5);
        wobbleGoal2.addStage(st, 1, 0.01);
        wobbleGoal2.addCustomOnce(new CodeSeg() {
            @Override
            public void run() {
                fastmode = true;
            }
        });
        wobbleGoal2.addStage(arm, 1, degreesToTicks(95));
        wobbleGoal2.addWaitUntil();
        wobbleGoal2.addStage(st, 0.8, 0.01);
        wobbleGoal2.addStage(arm, 1, degreesToTicks(180));
        wobbleGoal2.addStage(sgl, grabControl.getPos(0), 0.01);
        wobbleGoal2.addStage(sgr, grabControl.getPos(0), 0.5);
        wobbleGoal2.addStage(arm, 1, degreesToTicks(120));
    }

    public void defineGoback(){
        AutoModule back = new AutoModule();
        Path p1 = new Path(0,0,0);
        p1.addSetpoint(0,0,0);
        back.addPath(p1, this);
        goback = back;
    }


    public void resetAll(){
        odometry.reset(getLeftOdo(), getMiddleOdo(), getRightOdo());
        resetGyro();
    }

    public void update(){
        shooter.update();
        powerShot.update();
        wobbleGoal.update();
        wobbleGoal2.update();
        goback.update();
        outlController.updateMotorValues(getOutlPos());
        outrController.updateMotorValues(getOutrPos());
    }

    public boolean autoModulesRunning(){
        return (shooter.executing || powerShot.executing|| wobbleGoal.executing || wobbleGoal2.executing || goback.executing);
    }

    public boolean autoModulesPaused(){return  wobbleGoal.pausing || shooter.pausing || wobbleGoal2.pausing || powerShot.pausing || goback.pausing;}


    public double getArmPos(){
        return arm.getCurrentPosition()/NEV_DEGREES_TO_TICKS;
    }
    public double getArmVel(){
        double vel =  (getArmPos()-lastArmAngle);
        lastArmAngle = getArmPos();
        return vel;
    }

    public int degreesToTicks(double deg){
        return (int) (deg*NEV_DEGREES_TO_TICKS);
    }

    public boolean isArmInLimts(double dir){
        return limits.isInLimits(arm, dir, getArmPos());
    }

    public double getOutrPos(){
        return outr.getCurrentPosition()/GO_DEGREES_TO_TICKS;
    }
    public double getOutlPos(){
        return outl.getCurrentPosition()/GO_DEGREES_TO_TICKS;
    }



    public void outtakeWithEncoders(double speed){
        speed = speed*MAX_OUTTAKE_SPEED;
        outrController.setTargetSpeed(speed);
        outlController.setTargetSpeed(speed);
        double outrPow = outrController.getPow();
        double outlPow = outlController.getPow();
        outr.setPower(outrPow);
        outl.setPower(outlPow);
    }

    public void resetShooterIs(){
        outrController.reset();
        outlController.reset();
    }
    public void resetEncoders(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        in.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        in.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void resetArm(){
        if(resettingArm == 0){
            arm.setPower(-0.15);
            resettingArm++;
            timer.reset();
        }else if(resettingArm == 1) {
            if(getArmVel() == 0){
                if(timer.seconds() > 0.6){
                    arm.setPower(0);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    resettingArm++;
                }
            }else{
                timer.reset();
            }
        }
    }
    public boolean isDoneResettingArm(){
        return resettingArm > 1;
    }


    public double getLeftOdo(){
        return -in.getCurrentPosition();
    }
    public double getRightOdo(){
        return l2.getCurrentPosition();
    }
    public double getMiddleOdo(){
        return -r1.getCurrentPosition();
    }

    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        gyro.initialize(parameters);
    }

    public void resetGyro() {
        lastAngle = (int) gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        heading = 0;
    }

    public double getHeading() {
        double ca = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double da = ca - lastAngle;
        if (da < -180)
            da += 360;
        else if (da > 180)
            da -= 360;
        heading += da;
        lastAngle = ca;
        return heading;
    }

    public void startOdoThreadTele(){
        threadHandler.startTeleThread(new CodeSeg() {
            @Override
            public void run() {
                odometry.updateGlobalPosition(getLeftOdo(), getMiddleOdo(), getRightOdo(), getHeading());
            }
        }, 30);
    }
    public void stopOdoThreadTele() {
        threadHandler.stopTeleThread();
    }
    public void startOdoThreadAuto(LinearOpMode op){
        threadHandler.startAutoThread(new CodeSeg() {
            @Override
            public void run() {
                odometry.updateGlobalPosition(getLeftOdo(), getMiddleOdo(), getRightOdo(), getHeading());
            }
        }, op, 30);
    }
    public void stopOdoThreadAuto() {
        threadHandler.stopAutoThread();
    }

    public double getVoltage(){
        return expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
    }

    public double getVoltageScale(){
        return ((getVoltage()-12.5)*-0.07)+1;
        //0.367 - 14

    }


}