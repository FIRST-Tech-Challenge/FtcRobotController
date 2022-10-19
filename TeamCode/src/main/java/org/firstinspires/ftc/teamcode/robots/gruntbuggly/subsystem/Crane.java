package org.firstinspires.ftc.teamcode.robots.gruntbuggly.subsystem;

import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.util.Utils.servoNormalize;
import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.util.Constants.INCHES_PER_METER;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public static int SHOULDER_START_ANGLE = 0;
    public static int BULB_HOME_PWM = 1500;

    public static double SHOULDER_TICKS_PER_DEGREE = 15.7;  //from Proteus todo verify it works
    // This initial measurement is the range of motion from fully up (7.25 degrees from vertical) to horizontal, divided by that angle range
    // note the arm was moved by hand, not under motor power, and the angle encoder was not properly secured
    public static double SHOULDER_DEG_MIN = -10; // degrees down from horizontal - negative angles are counter clockwise while looking at the left side of the bot
    public static double SHOULDER_DEG_MAX = 90-7.25; //max elevation of shoulder when stalled up - measured by inclinometer
    public static double SHOULDER_DIRECT_TICKS_PER_DEGREE = (1937-88)/(SHOULDER_DEG_MAX); //todo verify/update when sensor secured and robot is more burned in - before tuning precision articulation
    public static double SHOULDER_TICK_MAX = 1849;

    public static double EXTEND_TICKS_PER_METER = 806/.2921; //todo verify this is still true

    public static double kF = 0.0;
    public static PIDCoefficients SHOULDER_PID = new PIDCoefficients(0.01, 0, 0);
    public static double SHOULDER_TOLERANCE = 1;
    public static double SHOULDER_POWER = 1.0;


    public static double kE = 0.0;
    public static PIDCoefficients EXTENDER_PID = new PIDCoefficients(0.01, 0, 0);
    public static double EXTENDER_TOLERANCE = 1;
    public static double EXTENDER_POWER = 1.0;
    public static double EXTENDER_TICS_MIN = 0;
    public static double EXTENDER_TICS_MAX = 3100; // of the robot
    boolean EXTENDER_CALIBRATE_MAX = false; //keep false except if calibrating EXTENDER_TICS_MAX

    public static double BULB_OPEN_POS = 1500;
    public static double BULB_CLOSED_POS = 1250;

    public static final double DISTANCE_SENSOR_TO_ELBOW = 0.33;
    public static final double GRIPPER_HEIGHT = 0.23;
    public static final double Y_LEEWAY = 0.05;
    public static final double X_LEEWAY = 0.02;
    public static final double ELBOW_HEIGHT = 0.24;
    public static final double CRANE_LENGTH = .3683;
    double extenderPwr = 1;
    double extendCorrection = 0;
    int extenderTargetPos = 0;
    private boolean extenderActivePID = true;
    private boolean shoulderActivePID = true;

    public static double kpExtender = 0.006; //proportional constant multiplier goodish
    public static  double kiExtender = 0.0; //integral constant multiplier
    public static  double kdExtender = 0.0;

    public static double kpShoulder = 0.002; //proportional constant multiplier goodish
    public static  double kiShoulder = 0.0; //integral constant multiplier
    public static  double kdShoulder = 0.0; //derivative constant multiplier

    public Servo bulbServo;
    public DcMotorEx extenderMotor;
    public DcMotorEx shoulderMotor;
    public DcMotorEx turretMotor;
    public DcMotor shoulderAngleEncoder;

    private PIDController shoulderPID;
    private PIDController extendPID;

    private boolean bulbGripped;

    private Robot robot;


    private Articulation articulation;

    boolean USE_MOTOR_SMOOTHING = true;

    public Crane(HardwareMap hardwareMap, Robot robot, boolean simulated) {
        this.robot = robot;
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
            extenderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
    double runShoulderAmp;
    double runExtendAmp;
    int extendMaxTics;

    public boolean calibrate(){
        // to calibrate we want the arm to be fully retracted and the shoulder
        // to be fully up at the physical stop as a repeatable starting position
        switch (calibrateStage) {
            case 0:
                //manually put arm down and extend by 1 slide before calibrating
                //shoulder all the way up and retract arm until they safely stall
                shoulderActivePID = false;
                extenderActivePID = false;
                shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shoulderMotor.setPower(.2); //move up at low power
                extenderMotor.setPower(-0.2);
                futureTime = futureTime(.25);
                calibrateStage++;
                break;

            case 1:
                if(System.nanoTime() > futureTime){
                    //sample low load amps
                    runShoulderAmp = shoulderMotor.getCurrent(CurrentUnit.AMPS);
                    runExtendAmp = extenderMotor.getCurrent(CurrentUnit.AMPS);
                    calibrateStage++;
                }
                break;

            case 2:
                // both motors are stalled
                if(shoulderMotor.getCurrent(CurrentUnit.AMPS) > 1 && extenderMotor.getCurrent(CurrentUnit.AMPS) > 1){
                    extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extenderMotor.setPower(-0.1); //barely stop it from extending
                    //enable PID on shoulder and rotate down to horizontal
                    shoulderMotor.setPower(0.0);
                    shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shoulderAngleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shoulderTargetPos = -(int)(SHOULDER_DEG_MAX*SHOULDER_DIRECT_TICKS_PER_DEGREE);
                    shoulderActivePID = true;
                    calibrateStage++;
                }
                break;

            case 3:
                if (shoulderDirectAnglePos>shoulderTargetPos-15 && shoulderDirectAnglePos<shoulderTargetPos+15){ //shoulder is horizontal, so reset encoder to begin from here - normally use shoulderPID.onTarget(), but that might not be setup correctly yet
                    shoulderActivePID = false;
                    shoulderAngleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //now horizontal should be at zero
                    shoulderTargetPos=0;
                    shoulderActivePID = true;
                    // WATCH OUT - begin extending arm - we want to find max extension (once established we will normally not do this)
                    if (EXTENDER_CALIBRATE_MAX) extenderMotor.setPower(.6);
                    shoulderTargetPos = 700;
                    calibrateStage++;
                }
                break;

            case 4:
                if (EXTENDER_CALIBRATE_MAX){
                    if (extenderMotor.getCurrent(CurrentUnit.AMPS) > 6 ){ //should be stalled at full extension
                        extendMaxTics = extenderMotor.getCurrentPosition();
                        calibrateStage++;
                    }
                }
                else
                    calibrateStage++;
                break;

            case 5: //enable extender PID to zero position - give 2 seconds to retract a good bit
                extenderMotor.setPower(0.0);
                extenderTargetPos = 0;
                extenderActivePID = true;
                futureTime = futureTime(0.5);
                calibrateStage++;
                break;

            case 6:
                if (System.nanoTime()>futureTime) {
                    shoulderTargetPos = 0;
                    calibrateStage = 0;
                    return true;
                }
                break;

        }
        return false;
    }

    double shoulderCorrection = 0;
    double shoulderPwr = 1;
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
        TEST_INIT(0, false),
        CAP(30, true);

        public double shoulderPos;
        public boolean bulbGripped;

        Articulation(double shoulderPos, boolean elbowPos) {
            this.shoulderPos = shoulderPos;
            this.bulbGripped = elbowPos;
        }
    }

    private int shoulderPosition = 0;
    private int shoulderDirectAnglePos = 0;
    private int extendPosition = 0;
    double shoulderAngle = 0;
    double extendMeters = 0;
    double shoulderAmps, extenderAmps;

    double shoulderHeight = 0;

    boolean inverseKinematic = false;
    double targetHeight = 0;
    double targetDistance = 0.43;
    double targetTurretAngle = 0;

    double targetX;
    double targetY;
    double targetZ;

    @Override
    public void update(Canvas fieldOverlay) {
        //todo - switch shoulderPosition to read the dedicated angle encoder
        shoulderPosition = shoulderMotor.getCurrentPosition();
        shoulderDirectAnglePos = shoulderAngleEncoder.getCurrentPosition();
        extendPosition = extenderMotor.getCurrentPosition();

        shoulderAngle = shoulderPosition / SHOULDER_TICKS_PER_DEGREE;
        extendMeters = extendPosition / EXTEND_TICKS_PER_METER;

        shoulderAmps = shoulderMotor.getCurrent(CurrentUnit.AMPS);
        extenderAmps = extenderMotor.getCurrent(CurrentUnit.AMPS);

        if(inverseKinematic){
            Pose2d robotPos = robot.driveTrain.getPoseEstimate();
            Pose2d turretPos = robot.turret.getTurretPosition(robotPos);
            targetTurretAngle = Math.atan2(targetY - turretPos.getY(),targetX-turretPos.getX());

            targetHeight = targetZ-shoulderHeight;

            targetDistance = Math.sqrt(Math.pow(targetY - turretPos.getY(),2) + Math.pow(targetX - turretPos.getX(),2));

            double angle = Math.atan(targetHeight/targetDistance);
            double length = Math.sqrt( Math.pow(targetHeight,2) + Math.pow(targetDistance,2) );
            setShoulderTargetDeg(angle);
            setExtendTargetDistance(length);
            robot.turret.setTargetHeading(targetTurretAngle);
        }

        if(shoulderActivePID)
            movePIDShoulder(kpShoulder, kiShoulder, kdShoulder, shoulderDirectAnglePos, shoulderTargetPos);
        else
            shoulderTargetPos = shoulderDirectAnglePos;

        if(extenderActivePID)
            movePIDExtend(kpExtender, kiExtender, kdExtender, extendPosition, extenderTargetPos);
        else
            extenderTargetPos = extendPosition;

        if(bulbGripped) {
            bulbServo.setPosition(servoNormalize(BULB_CLOSED_POS));
        }else {
            bulbServo.setPosition(servoNormalize(BULB_OPEN_POS));
        }
    }

    @Override
    public void stop() {
        setShoulderPwr(0);
        setextenderPwr(0);
        setShoulderActivePID(false);
        setextenderActivePID(false);
    }

    public void toggleGripper(){
        bulbGripped = !bulbGripped;
    }

    public void closeGripper(){
        bulbGripped = true;
    }
    public void openGripper(){
        bulbGripped = false;
    }
    public void setGripper(boolean g){
        bulbGripped = g;
    }

    @Override
    public String getTelemetryName() {
        return "Crane";
    }

    public void setextenderPwr(double pwr){ extenderPwr = pwr; }
    public void setextenderActivePID(boolean isActive){extenderActivePID = isActive;}
    public void setShoulderActivePID(boolean isActive){shoulderActivePID = isActive;}
    public void setShoulderTargetDeg(double deg){
        shoulderTargetPos = (int)(deg*SHOULDER_DIRECT_TICKS_PER_DEGREE);
    }
    public void setExtendTargetDistance(double dis){
        setExtendTargetPos((int)(dis*EXTEND_TICKS_PER_METER));
    }
    public void setShoulderPwr(double pwr){ shoulderPwr = pwr; }
    public  void setShoulderTargetPos(int t){ shoulderTargetPos = (int)(Math.max(Math.min(t,SHOULDER_TICK_MAX),0)); }
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
        setShoulderTargetPos(getShoulderPos() - (int)(100.0*speed));
    }

    public void increaseShoulderAngle(double speed){
        setShoulderTargetPos(getShoulderPos() + (int)(100.0*speed));
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
            telemetryMap.put("Calibrate Stage", calibrateStage);
            telemetryMap.put("Bulb Pos", bulbGripped);
            telemetryMap.put("Extend Meters", extendMeters);
            telemetryMap.put("Extend Tics", extendPosition);
            telemetryMap.put("Extend Amps", extenderAmps);
            telemetryMap.put("Extend Active PID", extenderActivePID);
            telemetryMap.put("Extend Target", extenderTargetPos);
            telemetryMap.put("Extend PID", extendCorrection);
            telemetryMap.put("Extend Run Amp", runExtendAmp);
            telemetryMap.put("Extend Max Tics", extendMaxTics);

            telemetryMap.put("Shoulder Angle", shoulderAngle);
            telemetryMap.put("Shoulder Tics", shoulderPosition);
            telemetryMap.put("Shoulder Power", shoulderMotor.getPower());
            telemetryMap.put("Shoulder Amps", shoulderAmps);
            telemetryMap.put("Shoulder Direct Angle Tics", shoulderDirectAnglePos);
            telemetryMap.put("Shoulder Target", shoulderTargetPos);
            telemetryMap.put("Shoulder Active", shoulderActivePID);
            telemetryMap.put("Shoulder PID Output", shoulderCorrection);
            telemetryMap.put("Running Amp", runShoulderAmp);

        }else{

        }
        return telemetryMap;
    }

}
