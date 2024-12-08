package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tatooine.utils.PIDFController;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

public class Wrist {
    private final double FRONT = 1;
    private final double BACK = 0;

    private double currentPos = 0;
    private Servo wristLeft = null;
    private Servo wristRight = null;
    private CRServo angleServo = null;
    private Telemetry telemetry;
    private boolean IS_DEBUG = false;

    private PIDFController pid = new PIDFController(0.01, 0, 0, 0);

    private final double ANGLE_TOLERANCE = 0.1;

    private AnalogInput angleSensor = null;

    private double offSet = 0;

    //TODO Put real valuesx (touch CAD)
    private final double FULL_RANGE = 291-24;

    private boolean shouldStayParralal = false;

    public Wrist(OpMode opMode, boolean IS_DEBUG) {
        this.wristLeft = opMode.hardwareMap.get(Servo.class, "wristLeft");
        this.wristRight = opMode.hardwareMap.get(Servo.class, "wristRight");
        this.angleServo = opMode.hardwareMap.get(CRServo.class, "angleServo");
        this.telemetry = opMode.telemetry;
        this.IS_DEBUG = IS_DEBUG;
        this.angleSensor = opMode.hardwareMap.get(AnalogInput.class, "angleSensor");
        if (IS_DEBUG) {
            telemetry.addData("Wrist", "Constructor");
        }
    }

    public Wrist(OpMode opMode) {
        this(opMode, false);
    }

    public void init() {
        wristLeft.setDirection(Servo.Direction.REVERSE);
        wristRight.setDirection(Servo.Direction.FORWARD);
        pid.setTolerance(ANGLE_TOLERANCE);
        angleServo.setPower(pid.calculate(getAngle(), 0));
        if (IS_DEBUG) {
            telemetry.addData("Wrist", "Init");
        }
    }

    public void changeState() {
        if (currentPos == FRONT) {
            wristLeft.setPosition(BACK);
            wristRight.setPosition(BACK);
            currentPos = BACK;
        } else {
            wristLeft.setPosition(FRONT);
            wristRight.setPosition(FRONT);
            currentPos = FRONT;
        }

        if (IS_DEBUG) {
            telemetry.addData("Wrist", "Change State");
            telemetry.addData("Current Pos", currentPos);
        }
    }

    public void open() {
        wristLeft.setPosition(FRONT);
        wristRight.setPosition(FRONT);
        currentPos = FRONT;
    }

    public void close() {
        wristLeft.setPosition(BACK);
        wristRight.setPosition(BACK);
        currentPos = BACK;
    }

    public double getAngle() {
        return MathUtil.voltageToDegrees(angleSensor.getVoltage());
    }

    public void setPostion(double postion) {
        wristLeft.setPosition(postion);
        wristRight.setPosition(postion);
    }


    public double getCurrentPos() {
        return currentPos;
    }

    public void setCurrentPos(double currentPos) {
        this.currentPos = currentPos;
    }

    public Servo getWristLeft() {
        return wristLeft;
    }

    public void setWristLeft(Servo wristLeft) {
        this.wristLeft = wristLeft;
    }

    public double getFRONT() {
        return FRONT;
    }

    public double getBACK() {
        return BACK;
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

    public void setIS_DEBUG(boolean IS_DEBUG) {
        this.IS_DEBUG = IS_DEBUG;
    }

    public Action moveToAngle(double angle) {
        return new MoveAngle(angle);
    }

    public Action specimen() {
        return new MoveAngle(90);
    }

    public void setShouldStayParralal(boolean shouldStayParralal) {
        this.shouldStayParralal = shouldStayParralal;
    }

    public class MoveAngle implements Action {

        double goal = 0;

        public MoveAngle(double goal) {
            pid.reset();
            this.goal = goal;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            angleServo.setPower(pid.calculate(getAngle(), goal));
            return !pid.atSetPoint();
        }
    }

    public class parralalToFloor implements Action {
        double armAngle = 0;

        public parralalToFloor(double armAngle) {
            this.armAngle = armAngle;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            offSet  = Math.cos(Math.toRadians(armAngle)) * 90 - FRONT;
            offSet /= FULL_RANGE;
            if (armAngle < 0) {
                offSet += 90/FULL_RANGE;
            }

            if (offSet > BACK){
                offSet = BACK;
            }
            else if (offSet < FRONT){
                offSet = FRONT;
            }
            setPostion(offSet);
            return shouldStayParralal;
        }
    }
}
