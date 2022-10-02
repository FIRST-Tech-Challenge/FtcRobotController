package org.firstinspires.ftc.teamcode.robots.LilVirani.subsystem;


import static org.firstinspires.ftc.teamcode.robots.LilVirani.util.Utils.servoNormalize;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.LilVirani.simulation.DcMotorExSim;

import org.firstinspires.ftc.teamcode.robots.LilVirani.simulation.ServoSim;

import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "FFCrane")
public class Crane implements Subsystem {
    public static int SHOULDER_START_ANGLE = 110;


    public static int BULB_HOME_PWM = 1500;


    public static double SHOULDER_TICKS_PER_DEGREE = 7.65;
    public static double BULB_PWM_PER_DEGREE = -600.0 / 90.0;


    public static double kF = 0.0;
    public static PIDCoefficients SHOULDER_PID = new PIDCoefficients(0.01, 0, 0);
    public static double SHOULDER_TOLERANCE = 1;
    public static double SHOULDER_POWER = 1.0;

    public static double SHOULDER_DEG_MIN = -90; // negative angles are counter clockwise while looking at the left side
    public static double SHOULDER_DEG_MAX = 90; // of the robot

    public static double BULB_OPEN_POS = -80;


    public static double BULB_CLOSED_POS = 140;


    public Servo bulbServo;
    public DcMotorEx shoulderMotor;


    private PIDController shoulderPID;


    private double shoulderPosition, shoulderAngle;
    private double shoulderTargetAngle;
    private int bulbPos = 0;


    private Articulation articulation;

    boolean USE_MOTOR_SMOOTHING = true;

    public Crane(HardwareMap hardwareMap, Turret turret, boolean simulated) {
        if (simulated) {
            shoulderMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            bulbServo = new ServoSim();
        } else {
            shoulderMotor = hardwareMap.get(DcMotorEx.class, "elbow");
            shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulderMotor.setTargetPosition(0);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bulbServo = hardwareMap.get(Servo.class, "servoGripper");
        }
        shoulderPID = new PIDController(SHOULDER_PID, (theta) -> kF * theta * Math.cos(shoulderAngle));
        shoulderPID.setInputRange(SHOULDER_DEG_MIN, SHOULDER_DEG_MAX);
        shoulderPID.setOutputRange(-1.0, 1.0);
        shoulderPID.setTolerance(SHOULDER_TOLERANCE);
        shoulderPID.enable();




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


    public void toggleBulb(){
        if(bulbPos == 1)
            bulbPos = 0;
        else if(bulbPos == 0)
            bulbPos = 1;
    }

    public void openBulb(){
        bulbPos = 1;
    }
    public void closeBulb(){
        bulbPos = 0;
    }


    @Override
    public void update(Canvas fieldOverlay) {




        if (shoulderTargetAngle > 180)
            shoulderTargetAngle -= 360;


        shoulderPosition = shoulderMotor.getCurrentPosition();
        shoulderAngle = SHOULDER_START_ANGLE + shoulderPosition / SHOULDER_TICKS_PER_DEGREE;

        shoulderMotor.setTargetPosition((int) ((shoulderTargetAngle - SHOULDER_START_ANGLE) * SHOULDER_TICKS_PER_DEGREE));
        shoulderMotor.setPower(SHOULDER_POWER);

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

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        telemetryMap.put("Current Articulation", articulation);

        if (debug) {
            telemetryMap.put("Shoulder Target Angle", shoulderTargetAngle);
            telemetryMap.put("Bulb Pos", bulbPos);

            telemetryMap.put("Shoulder Angle", shoulderAngle);
            telemetryMap.put("Shoulder Ticks", shoulderPosition);

        }


        return telemetryMap;
    }



}
