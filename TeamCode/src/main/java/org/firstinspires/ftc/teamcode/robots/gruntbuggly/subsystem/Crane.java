package org.firstinspires.ftc.teamcode.robots.gruntbuggly.subsystem;

import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.util.Utils.servoNormalize;
import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.util.Constants.INCHES_PER_METER;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.gruntbuggly.simulation.DcMotorExSim;

import org.firstinspires.ftc.teamcode.robots.gruntbuggly.simulation.ServoSim;

import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "PPCrane")
public class Crane implements Subsystem {
    public static int SHOULDER_START_ANGLE = 110;
    public static int BULB_HOME_PWM = 1500;
    public static double SHOULDER_TICKS_PER_DEGREE = 7.65;
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
    public static double EXTENDER_DEG_MIN = -90; // negative angles are counter clockwise while looking at the left side
    public static double EXTENDER_DEG_MAX = 90; // of the robot

    public static double BULB_OPEN_POS = 1500;
    public static double BULB_CLOSED_POS = 1250;


    public static final double DISTANCE_SENSOR_TO_ELBOW = 0.33;
    public static final double GRIPPER_HEIGHT = 0.23;
    public static final double Y_LEEWAY = 0.05;
    public static final double X_LEEWAY = 0.02;
    public static final double ELBOW_HEIGHT = 0.24;
    public static final double CRANE_LENGTH = .3683;
    double extendABobPwr = 0;
    double extendCorrection = 0;
    int extendABobTargetPos = 0;
    boolean extendABobActivePID = true;
    boolean shoulderActivePID = true;

    public static double kpExtendABob = 0.006; //proportional constant multiplier goodish
    public static  double kiExtendABob = 0.0; //integral constant multiplier
    public static  double kdExtendABob= 0.0;

    public static double kpElbow = 0.006; //proportional constant multiplier goodish
    public static  double kiElbow = 0.0; //integral constant multiplier
    public static  double kdElbow= 0.0; //derivative constant multiplier

    public Servo bulbServo;
    public DcMotorEx extendABob;
    public DcMotorEx shoulderMotor;
    public DcMotorEx turretMotor;


    private PIDController shoulderPID;
    private PIDController extendPID;

    private int bulbPos = 1;


    private Articulation articulation;

    boolean USE_MOTOR_SMOOTHING = true;

    public Crane(HardwareMap hardwareMap, Turret turret, boolean simulated) {
        extendABobTargetPos = 0;
        shoulderTargetPos = 0;
        if (simulated) {
            shoulderMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            extendABob = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            turretMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            bulbServo = new ServoSim();
        } else {
            shoulderMotor = hardwareMap.get(DcMotorEx.class, "elbow");
            extendABob = hardwareMap.get(DcMotorEx.class, "extender");
            turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
            shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulderMotor.setTargetPosition(0);
            extendABob.setTargetPosition(0);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extendABob.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bulbServo = hardwareMap.get(Servo.class, "servoGripper");
        }
        extendPID = new PIDController(0,0,0);
        shoulderPID = new PIDController(0,0,0);
        shoulderPID.setIntegralCutIn(40);
        shoulderPID.enableIntegralZeroCrossingReset(false);
    }

    int calibrateStage=0;
    double calibrateTimer;

    public boolean calibrate(){


        return true;
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

        //initialization of the PID calculator's output range, target value and multipliers
        extendPID.setOutputRange(-extendABobPwr, extendABobPwr);
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
        extendABob.setPower(extendCorrection);
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
    private int extendPosition = 0;
    double shoulderAngle = 0;
    double extendMeters = 0;

    @Override
    public void update(Canvas fieldOverlay) {
        shoulderPosition = shoulderMotor.getCurrentPosition();
        extendPosition = extendABob.getCurrentPosition();

        shoulderAngle = shoulderPosition / SHOULDER_TICKS_PER_DEGREE;
        extendMeters = extendPosition / EXTEND_TICKS_PER_METER;

        if(shoulderActivePID)
            movePIDShoulder(kpElbow, kiElbow, kdElbow, shoulderPosition, shoulderTargetPos);
        else
            shoulderTargetPos = shoulderPosition;


        if(extendABobActivePID)
            movePIDExtend(kpExtendABob, kiExtendABob, kdExtendABob, extendPosition, -extendABobTargetPos);
        else
            extendABobTargetPos = extendPosition;

        switch(bulbPos) {
                case 0:
                    bulbServo.setPosition(servoNormalize(BULB_CLOSED_POS));
                    break;
                case 1:
                    bulbServo.setPosition(servoNormalize(BULB_OPEN_POS));

         }
    }

    @Override
    public void stop() {

    }

    @Override
    public String getTelemetryName() {
        return "Crane";
    }

    public void setExtendABobPwr(double pwr){ extendABobPwr = pwr; }
    public void setExtendABobActivePID(boolean isActive){extendABobActivePID = isActive;}
    public void setShoulderPID(boolean isActive){shoulderActivePID = isActive;}
    public void setShoulderPwr(double pwr){ shoulderPwr = pwr; }
    public  void setShoulderTargetPos(int t){ shoulderTargetPos = t; }
    public  void setExtendTargetPos(int t){ extendABobTargetPos = t; }

    public int getExtendABobPos(){ return  extendPosition; }
    public int getShoulderPos(){ return  shoulderPosition; }
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
            telemetryMap.put("Extend Ticks", extendPosition);
            telemetryMap.put("Shoulder Angle", shoulderAngle);
            telemetryMap.put("Shoulder Ticks", shoulderPosition);

        }
        return telemetryMap;
    }

}
