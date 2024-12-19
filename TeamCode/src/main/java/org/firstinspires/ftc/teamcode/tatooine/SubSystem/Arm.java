package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tatooine.utils.PIDFController;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

public class Arm {

    //add variables

    //TODO change values to real ones (touch grass)
    private final double ANGLE_TOLERANCE = 2;//deg
    private final double EXTEND_TOLERANCE = 0.1;
    private final double EXTEND_CPR = 0;
    private final double ANGLE_CPR = 1425.1 * 3;
    private final double SPOOL_DIM = 0;//mm
    private final double AMP_LIMIT = 0;
    private final double angleOffSet = -26;
    private final boolean IS_DEBUG;
    private final double KF = 0.01;
    public Telemetry telemetry;
    private PIDFController anglePID = new PIDFController(0.03, 0, 0, 0);
    private PIDFController extendPID = new PIDFController(0.03, 0, 0, 0);
    private DcMotorEx angleLeft;
    private DcMotorEx angleRight;
    private DcMotorEx extendLeft;
    private DcMotorEx extendRight;
    private TouchSensor touchSensor = null;
    private double angleTimeout = 0;
    private double extendTimeout = 0;
    private double F = 0;


    //arm constructor
    public Arm(OpMode opMode, boolean IS_DEBUG) {

        telemetry = opMode.telemetry;

        this.IS_DEBUG = IS_DEBUG;
        if (IS_DEBUG) {
            opMode.telemetry.addData("second constructor", true);
        }

        angleLeft = (DcMotorEx) opMode.hardwareMap.get(DcMotor.class, "AL");
        angleRight = (DcMotorEx) opMode.hardwareMap.get(DcMotor.class, "AR");
        extendLeft = (DcMotorEx) opMode.hardwareMap.get(DcMotor.class, "EL");
        extendRight = (DcMotorEx) opMode.hardwareMap.get(DcMotor.class, "ER");
        touchSensor = opMode.hardwareMap.get(TouchSensor.class, "TS");

        init();


    }

    public Arm(OpMode opMode) {
        this(opMode, false);
    }


    //init function
    public void init() {
        //TODO change directions if needed
        angleLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        angleLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleRight.setDirection(DcMotorSimple.Direction.REVERSE);
        angleRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        extendLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendRight.setDirection(DcMotorSimple.Direction.FORWARD);
        extendRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        anglePID.setTolerance(ANGLE_TOLERANCE);
        extendPID.setTolerance(EXTEND_TOLERANCE);

        resetEncoders();
        if (IS_DEBUG) {
            telemetry.addData("Motor (AL) Direction", angleLeft.getDirection());
            telemetry.addData("Motor (AR) Direction", angleRight.getDirection());
            telemetry.addData("Motor (EL) Direction", extendLeft.getDirection());
            telemetry.addData("Motor (ER) Direction", extendRight.getDirection());
            telemetry.addData("resetEncoderInit", true);
        }
    }

    // resets all encoders
    public void resetEncoders() {
        if (IS_DEBUG) {
            telemetry.addData("resetEncoders", true);
        }
        resetAngleEncoder();
        resetExtendEncoders();
    }

    //resets the angle encoder
    public void resetAngleEncoder() {
        if (IS_DEBUG) {
            telemetry.addData("angleEncoderReset", true);
        }
        angleLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetExtendEncoders() {
        if (IS_DEBUG) {
            telemetry.addData("extendEncoderReset", true);
        }
        extendLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //returns the angle(transforms ticks to degrees)
    public double getAngle() {
        if (IS_DEBUG) {
            telemetry.addData("arm angle", (((MathUtil.convertTicksToDegries(ANGLE_CPR, angleLeft.getCurrentPosition()) + angleOffSet) + MathUtil.convertTicksToDegries(ANGLE_CPR, angleRight.getCurrentPosition()) + angleOffSet)) / 2);
        }
        return (((MathUtil.convertTicksToDegries(ANGLE_CPR, angleLeft.getCurrentPosition()) + angleOffSet) + MathUtil.convertTicksToDegries(ANGLE_CPR, angleRight.getCurrentPosition()) + angleOffSet)) / 2;
    }

    public double getExtend() {
        if (IS_DEBUG) {
            telemetry.addData("arm extend", true);
        }
        return (MathUtil.convertTicksToDistance(EXTEND_CPR, SPOOL_DIM, extendLeft.getCurrentPosition()) + MathUtil.convertTicksToDistance(EXTEND_CPR, SPOOL_DIM, extendRight.getCurrentPosition())) / 2;
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

    public DcMotorEx getAngleLeft() {
        return angleLeft;
    }

    public void setAngleLeft(DcMotorEx angleLeft) {
        this.angleLeft = angleLeft;
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

    public double getAngleOffSet() {
        return angleOffSet;
    }

    public PIDFController getExtendPID() {
        return extendPID;
    }

    public void setExtendPID(PIDFController extendPID) {
        this.extendPID = extendPID;
    }

    public DcMotorEx getAngleRight() {
        return angleRight;
    }

    public void setAngleRight(DcMotorEx angleRight) {
        this.angleRight = angleRight;
    }

    public DcMotorEx getExtendLeft() {
        return extendLeft;
    }

    public void setExtendLeft(DcMotorEx extendLeft) {
        this.extendLeft = extendLeft;
    }

    public DcMotorEx getExtendRight() {
        return extendRight;
    }

    public void setExtendRight(DcMotorEx extendRight) {
        this.extendRight = extendRight;
    }

    public double calculateF() {
        if (IS_DEBUG) {
            telemetry.addData("F", Math.cos(Math.toRadians(getAngle())) * KF);
        }
        return Math.cos(Math.toRadians(getAngle())) * KF;
    }

    //an actions that sets the angle of the arm to the desired angle
    public Action setAngle(double angle) {
        MoveAngle moveAngle = new MoveAngle(angle);
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

    public Action scoreSpecimenAction() {
        return new SequentialAction(setAngle(60), new SleepAction(3), setExtension(0), new SleepAction(3), setAngle(45));
    }

    public Action scoreSampleAction() {
        return new SequentialAction(setAngle(90), new SleepAction(3), setExtension(0));
    }

    public Action closeAction() {
        return new SequentialAction(setExtension(0), setAngle(-15));
    }

    public Action intakeAction() {
        return new SequentialAction(setAngle(0),setExtension(0));
    }


    public class moveExtension implements Action {
        private double goal = 0;

        public moveExtension(double goal) {
            this.goal = goal;
            extendPID.reset();
        }

        public double getGoal() {
            return goal;
        }

        public void setGoal(double goal) {
            this.goal = goal;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendLeft.setPower(extendPID.calculate(getExtend(), goal));
            extendRight.setPower(extendPID.calculate(getExtend(), goal));
            if (IS_DEBUG) {
                telemetry.addData("tiemout", extendPID.getTimeout());
                telemetry.addData("motor (EL) power", extendLeft.getPower());
                telemetry.addData("motor (ER) power", extendRight.getPower());
            }
            return !extendPID.atSetPoint();
        }
    }

    //Sets the angle motor power through PID and F
    public class MoveAngle implements Action {

        private double goal = 0;

        public MoveAngle(double goal) {
            anglePID.reset();
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
            //        if (touchSensor.isPressed()) {
//            resetAngleEncoder();
////        }
            double pidPower = anglePID.calculate(getAngle(), goal);
            if (pidPower < 0) {
                F = 0;
            } else {
                F = calculateF();
            }
            angleLeft.setPower(pidPower + F);
            angleRight.setPower(pidPower + F);
            if (IS_DEBUG) {
                telemetry.addData("runtime", anglePID.getRunTime());
                telemetry.addData("tiemout", anglePID.getTimeout());
                telemetry.addData("motor (AR) power", angleRight.getPower());
                telemetry.addData("motor (AL) power", angleLeft.getPower());
                telemetry.addData("motor (AL) power", angleLeft.getPower());
                telemetry.addData("pidPower+", pidPower);
                telemetry.addData("pidPower+F", pidPower + F);
            }
            return !anglePID.atSetPoint() && (F == 0 || pidPower < 0);
        }
    }
}
