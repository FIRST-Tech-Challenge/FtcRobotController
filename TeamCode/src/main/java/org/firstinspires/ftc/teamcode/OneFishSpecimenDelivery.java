package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OneFishSpecimenDelivery {

    private Servo pivot = null;
    private Servo claw = null;


    private LinearOpMode linearOpMode;

    public final double INTAKE_PITCH = 0.505;
    public final double DELIVERY_PITCH = 0.8;
    public final double VERTICAL_PITCH = 0.72;

    private final double SPECIMEN_SCORE_TIME = 850;

    public final double CLAW_CLOSE = 0.45;
    public final double CLAW_OPEN = 0.85;

    private double targetPosition;
    private double currentPosition;

    public final double RETRACT_TIMEOUT = 7;

    public boolean isDisabled = false;

    private double initAttempts = 0;

    private ElapsedTime time = new ElapsedTime();



    public OneFishSpecimenDelivery(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
    }


    private void Initialize(){
        try {
            pivot  = linearOpMode.hardwareMap.get(Servo.class, "specimenPivot");
            claw = linearOpMode.hardwareMap.get(Servo.class, "specimenClaw");
//            claw.scaleRange(0.0, 0.25);


        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find delivery.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public double getClawPosition() {return claw.getPosition();}

    public void setClawPosition(double p){ // 0-1
        claw.setPosition(p);
    }
    public void clawOpen(){
        setClawPosition(CLAW_OPEN);
    }
    public void clawClose(){
        setClawPosition(CLAW_CLOSE);
    }

    public double getPivotPosition() {return pivot.getPosition();}

    public void setPivotPosition(double p){ // 0-1
        pivot.setPosition(p);
    }
    public void pitchToIntake(){
        setPivotPosition(INTAKE_PITCH);
    }
    public void pitchToDelivery(){
        setPivotPosition(DELIVERY_PITCH);
    }
    public void pitchToVertical(){
        setPivotPosition(VERTICAL_PITCH);
    }

    public void setPivotTarget(double p, double timeElapsed){ // 0-1
//        pivot.setPosition();
        //-2*Math.pow(timeElapsed, 3)+3 * Math.pow(timeElapsed, 2)    GOES FROM 0 to 1 based on time in a cubic
    }

    public class ScoreSpecimenRR implements Action {
        private boolean initialized = false;
        private double time = 0.0;
        private ElapsedTime timer = new ElapsedTime();

        private double x, y;

        public ScoreSpecimenRR(double targetTime) {
            time = targetTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                initialized = true;
            }

            packet.addLine("In RR action");
            packet.addLine("Score Specimen");
            packet.put("Time Elapsed", timer.milliseconds());
            if (timer.milliseconds() < SPECIMEN_SCORE_TIME) {
                setPivotPosition(INTAKE_PITCH);
                clawClose();
                return true;
            }else if(timer.milliseconds() < time && timer.milliseconds() > SPECIMEN_SCORE_TIME) {
                clawOpen();
                return true;
            }else {
                return false;
            }
        }
    }
    public Action ScoreSpecimenAction(double targetTime) {
        return new OneFishSpecimenDelivery.ScoreSpecimenRR(targetTime);
    }

    public class PrepScoreSpecimenRR implements Action {
        private boolean initialized = false;
        private double time = 0.0;
        private double grab = 0.0;
        private ElapsedTime timer = new ElapsedTime();

        private double x, y;

        public PrepScoreSpecimenRR(double grabTime, double targetTime) {
            time = targetTime;
            grab = grabTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                initialized = true;
            }

            packet.addLine("In RR action");
            packet.addLine("Prep Score Specimen");
            packet.put("Time Elapsed", timer.milliseconds());
            if(timer.milliseconds() < grab){
                setPivotPosition(INTAKE_PITCH);
                clawOpen();
                return true;
            }else if(timer.milliseconds() < grab * 2){
                clawClose();
                return true;
            } else if (timer.milliseconds() < time && timer.milliseconds() > grab * 2) {
                setPivotPosition(DELIVERY_PITCH);
                clawClose();
                return true;
            } else {
                return false;
            }
        }
    }
    public Action PrepScoreSpecimenAction(double grabTime, double targetTime) {
        return new OneFishSpecimenDelivery.PrepScoreSpecimenRR(grabTime, targetTime);
    }

}
