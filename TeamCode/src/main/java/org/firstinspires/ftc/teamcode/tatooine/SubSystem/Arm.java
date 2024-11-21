package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tatooine.utils.PIDFController;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

public class Arm {

    //add variables

    //TODO change values to real ones (touch grass)
    private final double ANGLE_TOLERANCE = 2;//deg
    private final double EXTEND_TOLERANCE = 0.1;
    private final double EXTEND_CPR = 0;
    private final double ANGLE_CPR = 1425.1 * 3 ;
    private final double SPOOL_DIM = 0;//mm
    private final double AMP_LIMIT = 0;
    private final double angle = 0;
    private final double length = 0;
    private final double angleOffSet = -26;
    private PIDFController anglePID = new PIDFController(0.03, 0, 0, 0);
    private DcMotorEx angleMotor;
    private  Servo extendServoLeft;

    private Servo extendServoRight;

    public Telemetry telemetry;
    private TouchSensor touchSensor = null;

    private AnalogInput analogLeft = null;

    private AnalogInput analogRight = null;

    private final boolean IS_DEBUG;

    private double positionLeft;

    private double positionRight;

    private final double POS_LEFT_OFFSET = 269.7;
    private final double POS_RIGHT_OFFSET = 98.4;
    private final double OPEN_POSE_LEFT = 154.5;

    private final double OPEN_POSE_RIGHT = 166.5;

    private int level = 0;
    private ElapsedTime angleTimer = new ElapsedTime();
    private ElapsedTime extendTimer = new ElapsedTime();
    private double angleTimeout = 0;
    private double extendTimeout = 0;
    private final double KF = 0.01;
    private double F = 0;

    //TODO put value here
    private double openTimeWithPID = 8000;



    //arm constructor
    public Arm(OpMode opMode, boolean IS_DEBUG) {

        telemetry = opMode.telemetry;

        this.IS_DEBUG = IS_DEBUG;
        if (IS_DEBUG) {
            opMode.telemetry.addData("second constructor", true);
        }
        analogLeft = opMode.hardwareMap.get(AnalogInput.class ,"analogLeft");
        analogRight = opMode.hardwareMap.get(AnalogInput.class ,"analogRight");

        angleMotor = (DcMotorEx) opMode.hardwareMap.get(DcMotor.class, "AngleMotor");
        extendServoLeft = opMode.hardwareMap.get(Servo.class, "ExtendServoLeft");
        extendServoRight = opMode.hardwareMap.get(Servo.class, "ExtendServoRight");
        //touchSensor = opMode.hardwareMap.get(TouchSensor.class, "TouchSensor");

        anglePID.setTolerance(ANGLE_TOLERANCE);
        opMode.telemetry.update();
        init();


    }

//    public Arm(OpMode opMode, boolean isDebug) {
//        opMode.telemetry.addData("first constructor",true);
//        new Arm(opMode,false);
//    }

    //init function
    public void init() {
        //TODO change directions if needed
        //extendServoRight.setDirection(Servo.Direction.REVERSE);
        //extendServoLeft.setDirection(Servo.Direction.REVERSE);
        extendServoLeft.setPosition(0.1);
        extendServoRight.setPosition(0.1);
        angleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();
        if (IS_DEBUG){
            telemetry.addData("position left", getPositionLeft());
            telemetry.addData("position left", getPositionRight());
            telemetry.addData("Motor (A) Direction", angleMotor.getDirection());
            telemetry.addData("resetEncoderInit", true);
        }
    }

    // resets all encoders
    public void resetEncoders() {
        if (IS_DEBUG){
            telemetry.addData("angleEncoderReset", true);
        }
        resetAngleEncoder();
    }

    //resets the angle encoder
    public void resetAngleEncoder() {
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //returns the angle(transforms ticks to degrees)
    public double getAngle() {
        if (IS_DEBUG){
            telemetry.addData("arm angle", MathUtil.convertTicksToDegries(ANGLE_CPR, angleMotor.getCurrentPosition()) + angleOffSet);
        }
        return MathUtil.convertTicksToDegries(ANGLE_CPR, angleMotor.getCurrentPosition()) + angleOffSet;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public boolean isIS_DEBUG() {
        return IS_DEBUG;
    }

    public PIDFController getAnglePID() {
        return anglePID;
    }

    public void setAnglePID(PIDFController anglePID) {
        this.anglePID = anglePID;
    }

    public DcMotorEx getAngleMotor() {
        return angleMotor;
    }

    public void setAngleMotor(DcMotorEx angleMotor) {
        this.angleMotor = angleMotor;
    }

    public Servo getExtendServoLeft() {
        return extendServoLeft;
    }

    public void setExtendServoLeft(Servo extendServoLeft) {
        this.extendServoLeft = extendServoLeft;
    }

    public double getANGLE_TOLERANCE() {
        return ANGLE_TOLERANCE;
    }

    public double getEXTEND_TOLERANCE() {
        return EXTEND_TOLERANCE;
    }


    public TouchSensor getTouchSensor() {
        return touchSensor;
    }

    public void setTouchSensor(TouchSensor touchSensor) {
        this.touchSensor = touchSensor;
    }

    public double getAMP_LIMIT() {
        return AMP_LIMIT;
    }

    public double getSPOOL_DIM() {
        return SPOOL_DIM;
    }

    public double getANGLE_CPR() {
        return ANGLE_CPR;
    }

    public double getEXTEND_CPR() {
        return EXTEND_CPR;
    }

    public double getLength() {
        return length;
    }

    public Servo getExtendServoRight() {
        return extendServoRight;
    }

    public void setExtendServoRight(Servo extendServoRight) {
        this.extendServoRight = extendServoRight;
    }

    public AnalogInput getAnalogLeft() {
        return analogLeft;
    }

    public void setAnalogLeft(AnalogInput analogLeft) {
        this.analogLeft = analogLeft;
    }

    public AnalogInput getAnalogRight() {
        return analogRight;
    }

    public double getPositionRight() {
        if (IS_DEBUG){
            telemetry.addData("PositionLeft",(MathUtil.voltageToDegrees(analogRight.getVoltage())-POS_RIGHT_OFFSET)/ (OPEN_POSE_RIGHT));
        }
        return (MathUtil.voltageToDegrees(analogRight.getVoltage())-POS_RIGHT_OFFSET)/ (OPEN_POSE_RIGHT);
    }

    public void setPositionRight(double positionRight) {
        this.positionRight = positionRight;
    }

    public double getPositionLeft() {
        if (IS_DEBUG){
            telemetry.addData("PositionLeft",(MathUtil.voltageToDegrees(analogLeft.getVoltage())-POS_LEFT_OFFSET)/ (OPEN_POSE_LEFT));
        }
        return (MathUtil.voltageToDegrees(analogLeft.getVoltage())-POS_LEFT_OFFSET)/ (OPEN_POSE_LEFT);
    }

    public void setPositionLeft(double positionLeft) {
        this.positionLeft = positionLeft;
    }

    public double getAngleOffSet() {
        return angleOffSet;
    }

    public void setAnalogRight(AnalogInput analogRight) {
        this.analogRight = analogRight;
    }

    public double getF() {
        return F;
    }

    public void setF(double f) {
        F = f;
    }

    public double getKF() {
        return KF;
    }

    public double getExtendTimeout() {
        return extendTimeout;
    }

    public void setExtendTimeout(double extendTimeout) {
        this.extendTimeout = extendTimeout;
    }

    public double getAngleTimeout() {
        return angleTimeout;
    }

    public void setAngleTimeout(double angleTimeout) {
        this.angleTimeout = angleTimeout;
    }

    public ElapsedTime getExtendTimer() {
        return extendTimer;
    }

    public void setExtendTimer(ElapsedTime extendTimer) {
        this.extendTimer = extendTimer;
    }

    public ElapsedTime getAngleTimer() {
        return angleTimer;
    }

    public void setAngleTimer(ElapsedTime angleTimer) {
        this.angleTimer = angleTimer;
    }

    public int getLevel() {
        return level;
    }

    public void setLevel(int level) {
        this.level = level;
    }

    public double getOPEN_POSE_RIGHT() {
        return OPEN_POSE_RIGHT;
    }

    public double getOPEN_POSE_LEFT() {
        return OPEN_POSE_LEFT;
    }

    public double getPOS_RIGHT_OFFSET() {
        return POS_RIGHT_OFFSET;
    }

    public double getPOS_LEFT_OFFSET() {
        return POS_LEFT_OFFSET;
    }



    public double calculateF(){
        if (IS_DEBUG){
            telemetry.addData("F",Math.cos(Math.toRadians(getAngle()))* KF);
        }
        return Math.cos(Math.toRadians(getAngle()))* KF;
    }

    //an actions that sets the angle of the arm to the desired angle
    public Action setAngle(double angle) {
        MoveAngle moveAngle = new MoveAngle(angle);
        anglePID.reset();
        if (IS_DEBUG) {
            telemetry.addData("the new ang ", angle);
        }
        return moveAngle;
    }

    //an actions that sets the extension of the arm to the desired position
    public Action setExtension(double extension) {
        moveExtension move = new moveExtension(extension);
        if (IS_DEBUG) {
            telemetry.addData("the new extension ", extension);
        }
        return move;
    }

    public Action scoreAction (){
    return new SequentialAction(setAngle(60),new SleepAction(3), setExtension(0.8),new SleepAction(3),setAngle(45));
    }

    public Action closeAction (){
        return new SequentialAction(setExtension(0.1), setAngle(-15));
    }


    //Sets the Extension Servo position
    public class moveExtension implements Action {
        private double goal = 0;
        public moveExtension(double goal){
            this.goal = goal;
        }

        public double getGoal() {
            return goal;
        }

        public void setGoal(double goal) {
            this.goal = goal;
        }




        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            positionLeft = MathUtil.voltageToDegrees(analogLeft.getVoltage());
            positionRight = MathUtil.voltageToDegrees(analogRight.getVoltage());
            extendServoLeft.setPosition(goal);
            extendServoRight.setPosition(goal);
            if (IS_DEBUG) {
                telemetry.addData("servo Left(E) Position", extendServoLeft.getPosition());
                telemetry.addData("servo Right(E) Position", extendServoRight.getPosition());
                telemetry.addData("posLeft",getPositionLeft());
                telemetry.addData("posRight ",getPositionRight());
                telemetry.addData("goal",goal);
            }
            return !MathUtil.inTolerance(goal,getPositionRight(), EXTEND_TOLERANCE) && !MathUtil.inTolerance(goal,getPositionLeft(), EXTEND_TOLERANCE);
        }
    }
    //Sets the angle motor power through PID and F
    public class MoveAngle implements Action {

        private double goal = 0;

        private final double startAngle;

        private double angleDifference = 0;
        public MoveAngle(double goal)
        {
            anglePID.reset();
            telemetry.addData("did",true);
            this.goal = goal;
            this.startAngle = getAngle();
        }

        public double getGoal() {
            return goal;
        }

        public void setGoal(double goal) {
            this.goal = goal;
        }

        public double getAngleDifference() {
            if (IS_DEBUG){
                telemetry.addData("AngleDifference", Math.abs(goal- getStartAngle()));
            }
            return Math.abs(goal- getStartAngle());
        }

        public void setAngleDifference(double angleDifference) {
            this.angleDifference = angleDifference;
        }

        public double getStartAngle() {
            return startAngle;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //        if (touchSensor.isPressed()) {
//            resetAngleEncoder();
////        }
            anglePID.setTimeout(getAngleDifference()*(openTimeWithPID/90));
            double pidPower = anglePID.calculate(getAngle(), goal);
            F =0;
            angleMotor.setPower(pidPower+F);
            if (IS_DEBUG) {
                telemetry.addData("runtime", anglePID.getRunTime());
                telemetry.addData("tiemout", anglePID.getTimeout());
                telemetry.addData("startAngle", startAngle);
                telemetry.addData("motor (A) power", angleMotor.getPower());
                telemetry.addData("pidPower+",pidPower);
                telemetry.addData("pidPower+F",pidPower+F);
            }
            return !anglePID.atSetPoint();
        }
    }
}
