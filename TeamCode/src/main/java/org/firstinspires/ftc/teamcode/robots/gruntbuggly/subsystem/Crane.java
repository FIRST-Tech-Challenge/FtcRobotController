package org.firstinspires.ftc.teamcode.robots.gruntbuggly.subsystem;

import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.util.Utils.servoNormalize;
import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.util.Constants.INCHES_PER_METER;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.gruntbuggly.simulation.DcMotorExSim;

import org.firstinspires.ftc.teamcode.robots.gruntbuggly.simulation.ServoSim;

import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "PPCrane")
public class Crane implements Subsystem {
    public static int SHOULDER_START_ANGLE = 110;
    public static int BULB_HOME_PWM = 1500;

    public static double SHOULDER_TICKS_PER_DEGREE = 15.7;  //from Proteus todo verify it works
    // This initial measurement is the range of motion from fully up (7.25 degrees from vertical) to horizontal, divided by that angle range
    // note the arm was moved by hand, not under motor power, and the angle encoder was not properly secured
    public static double SHOULDER_DIRECT_TICKS_PER_DEGREE = (1937-88)/(90-7.25); //todo verify/update when sensor secured and robot is more burned in - before tuning precision articulation

    public static double EXTEND_TICKS_PER_METER = 806/.2921; //todo verify this is still true

    public static double kF = 0.0;
    public static PIDCoefficients SHOULDER_PID = new PIDCoefficients(0.01, 0, 0);
    public static double SHOULDER_TOLERANCE = 1;
    public static double SHOULDER_POWER = 1.0;
    public static double SHOULDER_DEG_MIN = -90; // negative angles are counter clockwise while looking at the left side
    public static double SHOULDER_DEG_MAX = 90; // of the robot

    public static double kE = 0.0;
    public static PIDCoefficients EXTENDER_PID = new PIDCoefficients(0.01, 0, 0);
    public static double EXTENDER_TOLERANCE = 1;
    public static double EXTENDER_POWER = 1.0;
    public static double EXTENDER_TICS_MIN = 0;
    public static double EXTENDER_TICS_MAX = 90; // of the robot

    public static double BULB_OPEN_POS = 1500;
    public static double BULB_CLOSED_POS = 1250;

    public static final double DISTANCE_SENSOR_TO_ELBOW = 0.33;
    public static final double GRIPPER_HEIGHT = 0.23;
    public static final double Y_LEEWAY = 0.05;
    public static final double X_LEEWAY = 0.02;
    public static final double ELBOW_HEIGHT = 0.24;
    public static final double CRANE_LENGTH = .3683;
    double extenderPwr = 0;
    double extendCorrection = 0;
    int extenderTargetPos = 0;
    private boolean extenderActivePID = true;
    private boolean shoulderActivePID = true;

    public static double kpExtender = 0.006; //proportional constant multiplier goodish
    public static  double kiExtender = 0.0; //integral constant multiplier
    public static  double kdExtender = 0.0;

    public static double kpElbow = 0.006; //proportional constant multiplier goodish
    public static  double kiElbow = 0.0; //integral constant multiplier
    public static  double kdElbow= 0.0; //derivative constant multiplier

    public Servo bulbServo;
    public DcMotorEx extenderMotor;
    public DcMotorEx shoulderMotor;
    public DcMotorEx turretMotor;
    public DcMotor shoulderAngleEncoder;

    private PIDController shoulderPID;
    private PIDController extendPID;

    private int bulbPos = 1;


    private Articulation articulation;

    boolean USE_MOTOR_SMOOTHING = true;

    public Crane(HardwareMap hardwareMap, Turret turret, boolean simulated) {
        extenderTargetPos = 0;
        shoulderTargetPos = 0;
        if (simulated) {
            shoulderMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            extenderMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            turretMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            shoulderAngleEncoder = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            bulbServo = new ServoSim();
        } else {
            shoulderMotor = hardwareMap.get(DcMotorEx.class, "shoulder");
            shoulderAngleEncoder = hardwareMap.get(DcMotorEx.class, "shoulderAngleEncoder"); //just a REV shaft encoder - no actual motor
            extenderMotor = hardwareMap.get(DcMotorEx.class, "extender");
            turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
            shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extenderMotor.setTargetPosition(0);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bulbServo = hardwareMap.get(Servo.class, "servoGripper");
        }
        extendPID = new PIDController(0,0,0);
        shoulderPID = new PIDController(0,0,0);
        shoulderPID.setIntegralCutIn(40);
        shoulderPID.enableIntegralZeroCrossingReset(false);
    }

    int calibrateStage=0;
    double futureTime;
    double runAmp;
    double runExtendAmp;

    public boolean calibrate(){
        //to calibrate we want the arm to be fully retracted and the shoulder
        // to be fully up at the physical stop as a repeatable starting position
        switch (calibrateStage) {
            case 0:
                //shoulder all the way up until it safely stalls
                shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shoulderMotor.setPower(.2); //move up at low power
                extenderMotor.setPower(0.3);
                futureTime = futureTime(.5);
                calibrateStage++;
                break;

            case 1:
                if(System.nanoTime() > futureTime){
                    runAmp = shoulderMotor.getCurrent(CurrentUnit.AMPS);
                    runExtendAmp = extenderMotor.getCurrent(CurrentUnit.AMPS);
                    calibrateStage++;
                }
                break;

            case 2:
                if(shoulderMotor.getCurrent(CurrentUnit.AMPS) > 2 && extenderMotor.getCurrent(CurrentUnit.AMPS) > 2){
                    calibrateStage++;
                }
                break;

            case 3:
                shoulderMotor.setPower(0.0);
                shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shoulderAngleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shoulderActivePID = true;
                extenderActivePID = true;
                shoulderTargetPos = 500;
                return true;

        }
        return false;
    }

    double shoulderCorrection = 0;
    double shoulderPwr = 0;
    int shoulderTargetPos = 0;

    public void movePIDShoulder(double Kp, double Ki, double Kd, double currentTicks, double targetTicks) {

        //initialization of the PID calculator's output range, target value and multipliers
        shoulderPID.setOutputRange(-shoulderPwr, shoulderPwr);
        shoulderPID.setPID(Kp, Ki, Kd);
        shoulderPID.setSetpoint(targetTicks);
        shoulderPID.enable();

        //initialization of the PID calculator's input range and current value
        shoulderPID.setInput(currentTicks);

        //calculates the correction to apply
        shoulderCorrection = shoulderPID.performPID();

        //moves elbow with the correction applied
        shoulderMotor.setPower(shoulderCorrection);
    }

    public void movePIDExtend(double Kp, double Ki, double Kd, double currentTicks, double targetTicks) {

        //todo - probably don't need our own PID - can use built in PID
        //initialization of the PID calculator's output range, target value and multipliers
        extendPID.setOutputRange(-extenderPwr, extenderPwr);
        extendPID.setPID(Kp, Ki, Kd);
        extendPID.setSetpoint(targetTicks);
        extendPID.enable();

        //initialization of the PID calculator's input range and current value
        //extendPID.setInputRange(0, 360);
        //extendPID.setContinuous();
        extendPID.setInput(currentTicks);

        //calculates the correction to apply
        extendCorrection = extendPID.performPID();

        //performs the extension with the correction applied
        extenderMotor.setPower(extendCorrection);
    }

    public enum Articulation {
        TEST_INIT(0, 0),
        CAP(30, 140);

        public double shoulderPos, bulbPos;

        Articulation(double shoulderPos, double elbowPos) {
            this.shoulderPos = shoulderPos;
            this.bulbPos = elbowPos;
        }
    }

    private int shoulderPosition = 0;
    private int shoulderDirectAnglePos = 0;
    private int extendPosition = 0;
    double shoulderAngle = 0;
    double extendMeters = 0;
    double shoulderAmps, extenderAmps;

    @Override
    public void update(Canvas fieldOverlay) {
        //todo - switch shoulderPosition to read the dedicated angle encoder
        shoulderPosition = shoulderMotor.getCurrentPosition();
        shoulderDirectAnglePos = shoulderAngleEncoder.getCurrentPosition();
        extendPosition = extenderMotor.getCurrentPosition();

        shoulderAngle = shoulderPosition / SHOULDER_TICKS_PER_DEGREE;
        extendMeters = extendPosition / EXTEND_TICKS_PER_METER;

        shoulderAmps= shoulderMotor.getCurrent(CurrentUnit.AMPS);
        extenderAmps= extenderMotor.getCurrent(CurrentUnit.AMPS);

        if(shoulderActivePID)
            movePIDShoulder(kpElbow, kiElbow, kdElbow, shoulderDirectAnglePos, shoulderTargetPos);
        else
            shoulderTargetPos = shoulderPosition;

        if(extenderActivePID)
            movePIDExtend(kpExtender, kiExtender, kdExtender, extendPosition, -extenderTargetPos);
        else
            extenderTargetPos = extendPosition;

        switch(bulbPos) {
                case 0:
                    bulbServo.setPosition(servoNormalize(BULB_CLOSED_POS));
                    break;
                case 1:
                    bulbServo.setPosition(servoNormalize(BULB_OPEN_POS));
                    break;

         }
    }

    @Override
    public void stop() {
        setShoulderPwr(0);
        setextenderPwr(0);
        setShoulderActivePID(false);
        setextenderActivePID(false);
    }

    @Override
    public String getTelemetryName() {
        return "Crane";
    }

    public void setextenderPwr(double pwr){ extenderPwr = pwr; }
    public void setextenderActivePID(boolean isActive){extenderActivePID = isActive;}
    public void setShoulderActivePID(boolean isActive){shoulderActivePID = isActive;}
    public void setShoulderPwr(double pwr){ shoulderPwr = pwr; }
    public  void setShoulderTargetPos(int t){ shoulderTargetPos = t; }
    public  int getShoulderTargetPos(){ return shoulderTargetPos; }
    public  void setExtendTargetPos(int t){ extenderTargetPos = t; }
    public boolean nearTargetShoulder(){
        if ((Math.abs( getShoulderPos()-getShoulderTargetPos()))<55) return true;
        else return false;
    }

    public void adjustShoulderAngle(double speed){
        setShoulderTargetPos(Math.max(getShoulderPos() + (int)(100 * speed), (int)(SHOULDER_DEG_MAX * SHOULDER_TICKS_PER_DEGREE)));
    }
    public void adjustArmLength(double speed){
        setExtendTargetPos(Math.min(getextenderPos() + (int)(100 * speed), (int)(EXTENDER_TICS_MAX)));
    }

    public void decreaseShoulderAngle(double speed){
        setShoulderTargetPos(Math.max(getShoulderPos() - (int)(100*speed), 0));
    }
    public int getextenderPos(){ return  extendPosition; }
    public int getShoulderPos(){ return  shoulderDirectAnglePos; }
    public double getShoulderAngle(){ return shoulderAngle;}

    public double getExtendMeters(){return extendMeters;}
    public double getExtendInches(){return extendMeters * INCHES_PER_METER; }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        telemetryMap.put("Current Articulation", articulation);

        if (debug) {
            telemetryMap.put("Bulb Pos", bulbPos);
            telemetryMap.put("Extend Meters", extendMeters);
            telemetryMap.put("Extend Tics", extendPosition);
            telemetryMap.put("Extend Amps", extenderAmps);
            telemetryMap.put("Extend Active PID", extenderActivePID);
            telemetryMap.put("Extend Target", extenderTargetPos);
            telemetryMap.put("Extend PID", extendCorrection);
            telemetryMap.put("Extend Run Amp", runExtendAmp);

            telemetryMap.put("Shoulder Angle", shoulderAngle);
            telemetryMap.put("Shoulder Tics", shoulderPosition);
            telemetryMap.put("Shoulder Power", shoulderMotor.getPower());
            telemetryMap.put("Shoulder Amps", shoulderAmps);
            telemetryMap.put("Shoulder Direct Angle Tics", shoulderDirectAnglePos);
            telemetryMap.put("Shoulder Target", shoulderTargetPos);
            telemetryMap.put("Shoulder Active", shoulderActivePID);
            telemetryMap.put("Shoulder PID Output", shoulderCorrection);
            telemetryMap.put("Running Amp", runAmp);

        }else{

        }
        return telemetryMap;
    }

}
