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

    public final double MAX_OUTTAKE_SPEED = 200 * Math.PI; // rad/s
    public final double LR_TO_OUTTAKE = 0.2;
    public final double FR_TO_OUTTAKE = 0.2;

    public void init(HardwareMap hwMap) {

        l1 = hwMap.get(DcMotor.class, "l1");
        l2 = hwMap.get(DcMotor.class, "l2");
        r1 = hwMap.get(DcMotor.class, "r1");
        r2 = hwMap.get(DcMotor.class, "r2");

        outr = hwMap.get(DcMotorEx.class, "outr");
        outl = hwMap.get(DcMotorEx.class, "outl");

        in = hwMap.get(DcMotor.class, "in");
        rh = hwMap.get(CRServo.class, "rh");

        rp = hwMap.get(Servo.class, "rp");

        compassSensor = hwMap.get(ModernRoboticsI2cCompassSensor.class, "compass");

        lr = hwMap.get(ModernRoboticsI2cRangeSensor.class, "lr");
        fr = hwMap.get(ModernRoboticsI2cRangeSensor.class, "fr");

        l1.setPower(0);
        l2.setPower(0);
        r1.setPower(0);
        r2.setPower(0);

        outr.setPower(0);
        outl.setPower(0);

        in.setPower(0);
        rh.setPower(0);

        l1.setDirection(DcMotorSimple.Direction.FORWARD);
        l2.setDirection(DcMotorSimple.Direction.REVERSE);
        r1.setDirection(DcMotorSimple.Direction.REVERSE);
        r2.setDirection(DcMotorSimple.Direction.FORWARD);

        outl.setDirection(DcMotorSimple.Direction.FORWARD);
        outr.setDirection(DcMotorSimple.Direction.REVERSE);

        in.setDirection(DcMotorSimple.Direction.FORWARD);

        rh.setDirection(DcMotorSimple.Direction.FORWARD);

        l1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        in.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rp.setDirection(Servo.Direction.FORWARD);
        rp.setPosition(rpStart);

        compassSensor.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);

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
        double robotTheta = compassSensor.getDirection() * Math.PI/180;
        double leftDis = lr.getDistance(DistanceUnit.CM)/100 * Math.cos(robotTheta) + LR_TO_OUTTAKE;
        double frontDis = fr.getDistance(DistanceUnit.CM)/100 * Math.cos(robotTheta) + FR_TO_OUTTAKE;
        outtake(MAX_OUTTAKE_SPEED/autoAimer.calcSpeed(frontDis, leftDis));
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
