package developing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

import global.AngularPosition;
import global.Constants;
import telefunctions.Cycle;

public class TestRobot {

    public DcMotor r1;
    public DcMotor l1;
    public DcMotor r2;
    public DcMotor l2;
    public DcMotor outr;
    public DcMotor outl;

    public DcMotor in;

    public CRServo rh;

    public Servo rp;

    public Cycle pushControl = new Cycle(0.1, 0.25, 0.32); // 0.1, 0.27, 0.35

    public ModernRoboticsI2cRangeSensor lr;
    public ModernRoboticsI2cRangeSensor br;

    public FTCAutoAimer autoAimer = new FTCAutoAimer();

    public AngularPosition angularPosition = new AngularPosition();

    public boolean intaking = false;
    public boolean outtaking = false;

    public double rpStart = 0.1;

    public boolean fastMode = true;

    public AutoModule2 shooter = new AutoModule2(); // 0

    public ArrayList<AutoModule2> autoModule2s = new ArrayList<>();

    public ButtonController outtakeButtonController = new ButtonController();


    public void init(HardwareMap hwMap) {

        l1 = getMotor(hwMap, "l1", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2 = getMotor(hwMap, "l2", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1 = getMotor(hwMap, "r1", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2 = getMotor(hwMap, "r2", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        in = getMotor(hwMap, "in", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outr = getMotor(hwMap, "outr", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outl = getMotor(hwMap, "outl", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rh = getCRServo(hwMap, "rh", CRServo.Direction.FORWARD);
        rp = getServo(hwMap, "rp", Servo.Direction.FORWARD, rpStart);

        angularPosition.init(hwMap);

        lr = hwMap.get(ModernRoboticsI2cRangeSensor.class, "lr");
        br = hwMap.get(ModernRoboticsI2cRangeSensor.class, "br");

        shooter.addStage(rp, pushControl, 1 , 0.3);
        shooter.addStage(0.8, outl);
        shooter.addStage( 0.4, outr);
        shooter.addPause();
        for(int i = 0; i < 3; i++) {
            shooter.addStage(rp, pushControl, 2, 0.3);
            shooter.addStage(rp, pushControl, 1, 0.3);
        }
        shooter.addStage(rp, pushControl, 0, 0.3);
        autoModule2s.add(shooter);


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
    }

    public void pushRings(double pos){
        rp.setPosition(pos);
    }


    public void outtake(double p){
        outr.setPower(p);
        outl.setPower(p);
    }

    public void updateIntake(boolean left_bumper, boolean right_bumper) {
        if(right_bumper){
            intaking = true;
        }else if(left_bumper){
            intaking = false;
            intake(-0.5);
            rh.setPower(-0.5);
        }else if(intaking){
            intake(1);
            rh.setPower(1);
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

    public boolean areAutomodulesRunning(){
        for (AutoModule2 a:autoModule2s) {
            if (a.isExecuting()) { return true; }
        }
        return false;
    }
    public void stopAllAutomodules(){
        for (AutoModule2 a:autoModule2s) {
            a.stop();
        }
    }

    public void toggleOuttake(boolean in) {
        if (outtakeButtonController.isPressing(in)) {
            outtaking = !outtaking;
            autoAimer.resetOuttake(getLeftAngPos(), getRightAngPos());
        }
    }

    public double getRightAngPos(){
        return (outr.getCurrentPosition()/Constants.GOBUILDA1_Ticks)*Constants.pi2;
    }
    public double getLeftAngPos(){
        return (outl.getCurrentPosition()/Constants.GOBUILDA1_Ticks)*Constants.pi2;
    }

    public double getLeftDistance(){
        return lr.getDistance(DistanceUnit.CM);
    }
    public double getBackDistance(){
        return br.getDistance(DistanceUnit.CM);
    }

    public void outtakeWithCalculations() {
        if (outtaking) {
//            autoAimer.update(angularPosition.getHeadingGY(), lr.getDistance(DistanceUnit.METER), br.getDistance(DistanceUnit.METER));
            autoAimer.update(0, 1.25,1.5);
            outr.setPower(autoAimer.getOutrPow(getRightAngPos()));
            outl.setPower(autoAimer.getOutlPow(getLeftAngPos()));
            rh.setPower(-0.5);
        } else {
            outr.setPower(0);
            outl.setPower(0);
            if (!intaking) { rh.setPower(0); }
        }
        //remove when make automodule
    }

    public double getRobotToGoalAngle() {
        double robotTheta = angularPosition.getHeadingCS() * Math.PI/180;
        double x = autoAimer.getDisFromCenter(getLeftDistance()/100, robotTheta) - Constants.GOAL_FROM_LEFT;
        double y = autoAimer.getDisFromCenter(getBackDistance()/100, robotTheta);
        return Math.atan2(y, x);
    }
}
