package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
public class OneFishIntake {
    private DcMotor extension = null;
    private DcMotor intake = null;
    private Servo pitch = null;
    private TouchSensor limitSwitch = null;

    private LinearOpMode linearOpMode;

    public final double CLICKS_PER_DEGREE = 3.5;
    private final double CLICKS_PER_CM = 24.92788;
    private final int MM_PER_METER = 1000;

    public final int DELTA_EXTENSION = 1000;
    public int minExtension = -999999999;
    public int maxExtension = DELTA_EXTENSION;
    private final double MIN_PITCH = 0.5;
    private final double DOWN_PITCH = 1;
    private final double UP_PITCH = 0.5;
    private final double AWAY_PITCH = 0.0;
    private final double TRANSFER_PITCH = 0.09;
    private final double MAX_PITCH = 1.0;

    private final int TICK_LOW_POWER_DISTANCE = 50;

    private double targetLengthCM;
    private int targetLength;
    private double currentPosition;

    public final double TICK_STOP_THRESHOLD = 5;

    private ElapsedTime time = new ElapsedTime();



    public OneFishIntake(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
    }


    private void Initialize(){//4mm
        try {
            extension = linearOpMode.hardwareMap.get(DcMotor.class, "extension");
            extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            intake = linearOpMode.hardwareMap.get(DcMotor.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            pitch = linearOpMode.hardwareMap.get(Servo.class, "intakePitch");
            pitch.scaleRange(0.5, 0.815);

            extension.setTargetPosition(extension.getCurrentPosition());

            limitSwitch = linearOpMode.hardwareMap.get(TouchSensor.class, "touch");

        }catch(NullPointerException e){
            linearOpMode.telemetry.addLine("Couldn't find intake");
        }
    }


    public void setTargetLength(int ticks){
        targetLength = ticks;
        extension.setTargetPosition(Math.max(minExtension, Math.min(maxExtension, ticks)));
    }

    public void setExtensionPower(double power){
        int error = 0;
        int errorMult = 0;
        int current = extension.getCurrentPosition();
        extension.setPower(power);

        if(current > maxExtension /2){
            error = (current - maxExtension);
        } else {
            error = (current- minExtension);
        }

        errorMult = error/(TICK_LOW_POWER_DISTANCE*2);

        if(Math.abs(error) < TICK_LOW_POWER_DISTANCE){
            extension.setPower(power * errorMult);
            linearOpMode.telemetry.addLine("BRAKING");
        }
        linearOpMode.telemetry.addData("Intake Current: ", current);
    }

    public void runToPosition(){
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runPower(){
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void updateLength(){
        linearOpMode.telemetry.addData("Intake Length: ", targetLength);
        linearOpMode.telemetry.addData("Intake Current: ", extension.getCurrentPosition());
        if(Math.abs(extension.getCurrentPosition() - extension.getTargetPosition()) > TICK_STOP_THRESHOLD){
            extension.setPower(1);
        } else{
            extension.setPower(0);
        }
    }


    public void setIntakePower(float power){
        intake.setPower(power);
    }

    public void setPitch(float position){
        pitch.setPosition(position);
    }
    public double getPitch(){return pitch.getPosition();}

    public void pitchUp(){
        pitch.setPosition(UP_PITCH);
    }
    public void pitchDown(){
        pitch.setPosition(DOWN_PITCH);
    }
    public void pitchAway(){pitch.setPosition(AWAY_PITCH);}

    public void pitchToTransfer(){pitch.setPosition(TRANSFER_PITCH);}

    public int getExtensionTicks(){ return extension.getCurrentPosition();}

    public void limitCheck() {
        if (limitSwitch.isPressed()) {
            minExtension = extension.getCurrentPosition();
            maxExtension = minExtension+DELTA_EXTENSION;
        }
    }

    public class RunToLengthRR implements Action {
        private boolean initialized = false;
        private int target = 0;
        //timeout in MILLISECONDS
        private double timeout = 0.0;

        private ElapsedTime timer = new ElapsedTime();

        public RunToLengthRR(int targetLength, double timeoutTime) {
            target = targetLength;
            timeout = timeoutTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                setTargetLength(target);
                timer.reset();
            }

            packet.addLine("In RR action");
            packet.addLine("Drive For Time");
            packet.put("Time Elapsed", timer.milliseconds());
            if (Math.abs(extension.getCurrentPosition() - target) > 10 && timer.milliseconds() < timeout) {
                updateLength();
                return true;
            } else {
                setTargetLength(extension.getCurrentPosition());
                return false;
            }
        }
    }
    public Action RunToLengthAction(int targetLength, double timeoutTime) {
        return new OneFishIntake.RunToLengthRR(targetLength, timeoutTime);
    }


    public class SpinIntakeRR implements Action {
        private boolean initialized = false;
        //timeout in MILLISECONDS
        private double timeout = 0.0;

        private double power = 0.0;

        private ElapsedTime timer = new ElapsedTime();

        public SpinIntakeRR(double spinPower, double timeoutTime) {
            timeout = timeoutTime;
            power = spinPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                setIntakePower((float)power);
                timer.reset();
            }

            packet.addLine("In RR action");
            packet.addLine("Drive For Time");
            packet.put("Time Elapsed", timer.milliseconds());
            if (timer.milliseconds() < timeout) {
                return true;
            } else {
                setIntakePower(0);
                return false;
            }
        }
    }
    public Action SpinIntakeAction(double spinPower, double timeoutTime) {
        return new OneFishIntake.SpinIntakeRR(spinPower, timeoutTime);
    }


}
