package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
    private final double ANGLE_TOLERANCE = 1;//deg
    private final double EXTEND_TOLERANCE = 0.1;
    private final double EXTEND_CPR = 0;
    private final double ANGLE_CPR = 1425.1 * 5 ;
    private final double SPOOL_DIM = 0;//mm
    private final double AMP_LIMIT = 0;
    private final double angle = 0;
    private final double length = 0;
    private final double angleOffSet = -15;
    private PIDFController anglePID = new PIDFController(0.15, 0, 0, 0);
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
    private ElapsedTime timer = new ElapsedTime();




    //arm constructor
    public Arm(OpMode opMode, boolean IS_DEBUG) {

        telemetry = opMode.telemetry;

        this.IS_DEBUG = IS_DEBUG;
        opMode.telemetry.addData("second constructor",true);

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
        //extendMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        resetEncoders();
    }

    // resets all encoders
    public void resetEncoders() {
        resetAngleEncoder();
    }

    //resets the angle encoder
    public void resetAngleEncoder() {
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //returns the angle(transforms ticks to degrees)
    public double getAngle() {
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
        return (MathUtil.voltageToDegrees(analogRight.getVoltage())-POS_RIGHT_OFFSET)/ (OPEN_POSE_RIGHT);
    }

    public void setPositionRight(double positionRight) {
        this.positionRight = positionRight;
    }

    public double getPositionLeft() {
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

    //an actions that sets the angle of the arm to the desired angle
    public Action setAngle(double angle) {
         moveAngle move = new moveAngle(angle);

//        if (touchSensor.isPressed()) {
//            resetAngleEncoder();
//        }
        if (IS_DEBUG) {
            telemetry.addData("the new ang ", angle);
        }
        return new moveAngle(angle);
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
        ScoreAction scoreAction = new ScoreAction();
        return scoreAction;
    }


    public class ScoreAction implements Action{

        private boolean isStartedAgain;
        private double goal = 0;
        private double servoGoal = 0;

        public ScoreAction(){
            level = 0;
        }

        public boolean isStartedAgain() {
            return isStartedAgain;
        }

        public void setStartedAgain(boolean startedAgain) {
            isStartedAgain = startedAgain;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            positionLeft = MathUtil.voltageToDegrees(analogLeft.getVoltage());
            positionRight = MathUtil.voltageToDegrees(analogRight.getVoltage());
            if (level == 0) {goal = 40;}
            else if (anglePID.atSetPoint() && level == 0){;
                level = 1;
                goal = 60;
                servoGoal =1;}
            if (anglePID.atSetPoint() && level == 1) {angleMotor.setPower(0);}
            angleMotor.setPower(anglePID.calculate(getAngle(), goal));
            extendServoLeft.setPosition(servoGoal);
            extendServoRight.setPosition(servoGoal);
            return !anglePID.atSetPoint() && !MathUtil.inTolerance(goal,getPositionRight(), EXTEND_TOLERANCE) && !MathUtil.inTolerance(goal,getPositionLeft(), EXTEND_TOLERANCE) && level == 1;
        }
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
            angleMotor.setPower(anglePID.calculate(getAngle(), goal));
            if (IS_DEBUG) {
                telemetryPacket.put("motor (A) pos", angleMotor.getCurrentPosition());
                telemetry.addData("motor (A) pos", angleMotor.getCurrentPosition());
                telemetry.addData("motor (A) power", angleMotor.getPower());
            }
            if (anglePID.atSetPoint()){
                angleMotor.setPower(0);
            }
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
    public class moveAngle implements Action {

        public moveAngle (double goal)
        {
            telemetry.addData("did",true);
            this.goal = goal;
        }
        private double goal = 0;

        public double getGoal() {
            return goal;
        }

        public void setGoal(double goal) {
            this.goal = goal;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            angleMotor.setPower(anglePID.calculate(getAngle(), goal));
            if (IS_DEBUG) {
                telemetryPacket.put("motor (A) pos", angleMotor.getCurrentPosition());
                telemetry.addData("motor (A) pos", angleMotor.getCurrentPosition());
                telemetry.addData("motor (A) power", angleMotor.getPower());
            }
            if (anglePID.atSetPoint()){
                angleMotor.setPower(0);
            }
            return !anglePID.atSetPoint();
        }
    }

}
