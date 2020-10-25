package global;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.ForkJoinPool;

import telefunctions.AutoModule;
import telefunctions.Cycle;
import telefunctions.Limits;
import telefunctions.ServoController;
import telefunctions.SpeedController;


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

    public boolean intaking = false;
    public boolean outtaking = false;

    public double turnStart = 0.3;
    public double grabStart = 0.7;
    public double liftStart = 0.12;
    public double liftSecond = 0.27;
    public double shootStartR = 0.05;
    public double shootStartL = 0.1;
    public double intakeSpeed = 1;
    public double outtakeSpeed = 0.4;
    public double maxArmPos = 225;

    public final double NEVEREST256_TICKS = 7168;
    public final double NEV_DEGREES_TO_TICKS = NEVEREST256_TICKS/360;
    public final double GOBUILDA1_Ticks = 28;
    public final double GO_DEGREES_TO_TICKS = GOBUILDA1_Ticks/360;
    public final double MAX_OUTTAKE_SPEED = 32400;

    public Cycle grabControl = new Cycle(grabStart, 0.45);
    public Cycle liftControl = new Cycle(liftStart, liftSecond, 0.5);
    public Cycle shootControlR = new Cycle(shootStartR, 0.24, 0.25);
    public Cycle shootControlL = new Cycle(shootStartL, 0.15, 0.33);

    public ServoController turnControl = new ServoController(turnStart, 0.0, 0.7);

    public AutoModule shooter = new AutoModule();
    public AutoModule wobbleGoal = new AutoModule();

    public Limits limits = new Limits();


    public SpeedController outrController = new SpeedController(0.5, 0.01, 0.05);
    public SpeedController outlController = new SpeedController(0.5, 0.01, 0.05);







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

        resetEncoders();

        defineShooter();
        defineWobbleGoal();


        limits.addLimit(arm, 0, maxArmPos);





    }

    public void move(double f, double s, double t){
        l1.setPower(f-s-t);
        l2.setPower(-f-s+t);
        r1.setPower(f+s+t);
        r2.setPower(-f+s-t);
    }


    public void intake(double p){
        in.setPower(p);
    }

    public void outtake(double p){
        outr.setPower(p);
        outl.setPower(p);
    }

    public void turnArm(double p){
        arm.setPower(p);
    }

    public void lift(double pos){
        slr.setPosition(pos+0.07);
        sll.setPosition(pos);
    }

    public void shoot(double pr, double pl){
        ssr.setPosition(pr);
        ssl.setPosition(pl);
    }

    public void turnWobbleArm(double pos){
        st.setPosition(pos);
    }

    public void grab(double pos){
        sgr.setPosition(pos);
        sgl.setPosition(pos);
    }

    public void defineShooter(){
        shooter.addStage(outl, outtakeSpeed, 0.01);
        shooter.addStage(outr, outtakeSpeed, 0.01);
        shooter.addStage(slr, liftSecond+0.07, 0.01);
        shooter.addStage(sll, liftSecond, 0.7);
        for(int i = 0; i < 3;i++) {
            shooter.addStage(ssr, shootControlR.getPos(1), 0.01);
            shooter.addStage(ssl, shootControlL.getPos(1), 0.4);
            shooter.addStage(ssr, shootControlR.getPos(2), 0.01);
            shooter.addStage(ssl, shootControlL.getPos(2), 0.4);
        }
    }
    public void defineWobbleGoal(){
        wobbleGoal.addStage(arm,  0.5, degreesToTicks(200));
        wobbleGoal.addStage(st, 0.65, 0.7);
        wobbleGoal.addWaitUntil();
        wobbleGoal.addStage(sgl, grabControl.getPos(1), 0.01);
        wobbleGoal.addStage(sgr, grabControl.getPos(1), 0.7);
        wobbleGoal.addStage(arm,  0.5, degreesToTicks(225));
        wobbleGoal.addStage(st, 0.16, 0.7);
        wobbleGoal.addWaitUntil();
        wobbleGoal.addStage(slr, liftControl.getPos(2)+0.07, 0.01);
        wobbleGoal.addStage(sll, liftControl.getPos(2), 0.7);

    }

    public void update(){
        shooter.update();
        wobbleGoal.update();
        outlController.updateMotorValues(getOutlPos());
        outrController.updateMotorValues(getOutrPos());
    }

    public boolean autoModulesRunning(){
        return shooter.executing || wobbleGoal.executing;
    }

    public boolean autoModulesPaused(){return  wobbleGoal.pausing;}


    public double getArmPos(){
        return arm.getCurrentPosition()/NEV_DEGREES_TO_TICKS;
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

    public void resetEncoders(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}