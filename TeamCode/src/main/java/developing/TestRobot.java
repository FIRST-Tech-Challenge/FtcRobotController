package developing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    public Cycle pushControl = new Cycle(0.1, 0.3);

    public ModernRoboticsI2cCompassSensor compassSensor;

    public ModernRoboticsI2cRangeSensor lr;
    public ModernRoboticsI2cRangeSensor fr;

    public FTCAutoAimer autoAimer = new FTCAutoAimer();

    public boolean intaking = false;
    public boolean calibratingCompass = true;

    public double rpStart = 0.1;

//    public final double MAX_OUTTAKE_SPEED = 200 * Math.PI; // rad/s
//    public final double LR_TO_OUTTAKE = 0.2;
//    public final double FR_TO_OUTTAKE = 0.2;



    public void init(HardwareMap hwMap) {


        l1 = getMotor(hwMap, "l1", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2 = getMotor(hwMap, "l2", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1 = getMotor(hwMap, "r1", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2 = getMotor(hwMap, "r2", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        in = getMotor(hwMap, "in", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outr = getMotor(hwMap, "outr", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outl = getMotor(hwMap, "outl", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rh = getCRServo(hwMap, "rh", CRServo.Direction.FORWARD);
        rp = getServo(hwMap, "rp", Servo.Direction.FORWARD, rpStart);


        compassSensor = hwMap.get(ModernRoboticsI2cCompassSensor.class, "compass");

        lr = hwMap.get(ModernRoboticsI2cRangeSensor.class, "lr");
        fr = hwMap.get(ModernRoboticsI2cRangeSensor.class, "fr");

        compassSensor.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);

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
        crServo.setDirection(CRServo.Direction.FORWARD);
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
    }

    public void pushRings(double pos){
        rp.setPosition(pos);
    }

    public void outtakeWithCalculations() {
        double p = autoAimer.getOuttakePower(compassSensor.getDirection(), lr.getDistance(DistanceUnit.METER), fr.getDistance(DistanceUnit.METER));
        outr.setPower(p);
        outl.setPower(p);

    }

    public double getRobotToGoalAngle() {
        double robotTheta = compassSensor.getDirection() * Math.PI/180;
        double x = autoAimer.getDisFromCenter(lr.getDistance(DistanceUnit.METER), robotTheta) - Constants.GOAL_FROM_LEFT;
        double y = autoAimer.getDisFromCenter(fr.getDistance(DistanceUnit.METER), robotTheta);
        return Math.atan2(y, x);
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
        }else if(intaking){
            intake(1);
        }else{
            intake(0);
        }
    }

    public void setCompassMode () {
        if (calibratingCompass && !compassSensor.isCalibrating()) {
            compassSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
            calibratingCompass = false;
        }
    }

}
