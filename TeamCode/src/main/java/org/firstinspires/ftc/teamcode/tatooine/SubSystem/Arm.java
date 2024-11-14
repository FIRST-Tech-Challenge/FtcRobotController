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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tatooine.utils.PIDFController;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

public class Arm {

    //add variables

    //TODO change values to real ones (touch grass)
    private final double ANGLE_TOLERANCE = 1;//deg
    private final double EXTEND_TOLERANCE = 0;//mm
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
    private boolean isDebug;

    private double positionLeft;

    private double positionRight;
    private final double POS_LEFT_OFFSET = 269.7;
    private final double POS_RIGHT_OFFSET = 98.4;
    private final double OPEN_POSE_LEFT = 154.5;
    private final double OPEN_POSE_RIGHT = 166.5;
    private final double extendTolerance = 0.1;



    //arm constructor
    public Arm(OpMode opMode, boolean isDebug) {

        telemetry = opMode.telemetry;

        this.isDebug = isDebug;
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

    public Arm(OpMode opMode) {
        opMode.telemetry.addData("first constructor",true);
        new Arm(opMode,false);
    }

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

    public boolean isDebug() {
        return isDebug;
    }

    public void setDebug(boolean debug) {
        isDebug = debug;
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
        return MathUtil.voltageToDegrees(analogRight.getVoltage())-POS_RIGHT_OFFSET;
    }

    public void setPositionRight(double positionRight) {
        this.positionRight = positionRight;
    }

    public double getPositionLeft() {
        return MathUtil.voltageToDegrees(analogLeft.getVoltage())-POS_LEFT_OFFSET;
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
        if (isDebug) {
            telemetry.addData("the new ang ", angle);
        }
        return new moveAngle(angle);
    }

    //an actions that sets the extension of the arm to the desired position
    public Action setExtension(double extension) {
        moveExtension move = new moveExtension(extension);
        //telemetry.addData("the new extension ", extension);
        return move;
    }

    //Sets the Extension Servo position
    public class moveExtension implements Action {
        private double goal = 0;
        private double leftPosAfter;
        private double rightPosRight;
        public moveExtension(double goal){
            this.goal = goal;
        }


        public double getGoal() {
            return goal;
        }

        public void setGoal(double goal) {
            this.goal = goal;
        }

        public double getRightPosAfter() {
            if (goal == 0) {
                return 0;
            } else {
                return getPositionRight() / (OPEN_POSE_RIGHT / goal);
            }
        }
        public double getLeftPosAfter() {
            if (goal == 0) {
                return 0;
            } else {
                return getPositionLeft() / (OPEN_POSE_LEFT / goal);
            }
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            positionLeft = MathUtil.voltageToDegrees(analogLeft.getVoltage());
            positionRight = MathUtil.voltageToDegrees(analogRight.getVoltage());
            extendServoLeft.setPosition(goal);
            extendServoRight.setPosition(goal);
            if (isDebug) {
                telemetryPacket.put("servo (E) Position", extendServoLeft.getPosition());
                telemetry.addData("servo (E) Position", extendServoLeft.getPosition());
                telemetry.addData("posLeft",getPositionLeft());
                telemetry.addData("posRight ",getPositionRight());
                telemetry.addData("return", !MathUtil.inTolerance(goal, getRightPosAfter(),extendTolerance));
                telemetry.addData("getRightPosAfter",getPositionRight() / (OPEN_POSE_RIGHT / goal));
                telemetry.addData("goal",goal);
            }
            return !MathUtil.inTolerance(goal,getRightPosAfter(),extendTolerance);
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
            if (isDebug) {
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
