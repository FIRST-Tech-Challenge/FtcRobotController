package global;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ForkJoinPool;


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
    public double liftStart = 0.0;
    public double shootStart = 0.0;
    public double intakeSpeed = 1;
    public double outtakeSpeed = 0.35;



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
        ssr.setPosition(shootStart);
        ssl.setPosition(1-shootStart);
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
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    public void moveArm(double p){
        arm.setPower(p);
    }

    public void lift(double pos){
        slr.setPosition(pos+0.05);
        sll.setPosition(pos);
    }

    public void shoot(double pos){
        ssr.setPosition(pos);
        ssl.setPosition(pos);
    }

    public void turnArm(double pos){
        st.setPosition(pos);
    }

    public void grab(double pos){
        sgr.setPosition(pos);
        sgl.setPosition(pos);
    }



}