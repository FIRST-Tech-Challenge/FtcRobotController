package org.firstinspires.ftc.teamcode.Mechanisms.Extension;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Actuators.DcMotorAdvanced;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Encoder;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.FeedForward;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Planners.MotionProfile;

public class Extension2 {
    HardwareMap hardwareMap;
    FeedForward feedForward;
    PID pid;
    double motorPower;
    public int currentPosition = 0;
    public DcMotorAdvanced extensionMotor;
    public Encoder encoder;
    TouchSensor limiter;
    public static double kA=0.2;
    public static double kV=0.2;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double extensionThreshold = 0.1;
    double spoolRadius =  0.702; // [in]
    public static double maxAcceleration = 5;
    public static double maxDeceleration = 5;
    public static double maxVelocity = 6;
    private final double ticksPerRev = 384.5;
    private final double ticksPerInch = ticksPerRev / (2 * Math.PI * spoolRadius);
    boolean reverse;
    public static double maxVoltage = 12.5;


    public Extension2(HardwareMap hardwareMap, Battery battery){
        this.hardwareMap = hardwareMap;

        this.extensionMotor = new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, "extensionMotor"), battery, maxVoltage);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, "extensionMotor"));
        this.extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.extensionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.extensionMotor.setPower(0);

        this.currentPosition = 0;
        this.feedForward = new FeedForward(kV, kA, 0);
        this.pid = new PID(kP, kI, kD, PID.functionType.LINEAR);
        this.limiter = hardwareMap.get(TouchSensor.class, "extensionTouch");
    }
    private void checkLimit(){
        if (limiter.isPressed()){
            encoder.reset();
        }
    }

    public double ticksToInches(double ticks){
        return ticks/ticksPerInch;
    }

    // See above but rename the parameter to targetHeight
    public Action moveToHeight(double targetHeight) {

        return new Action() {
            private double initialPos;
            private boolean reverse;
            private MotionProfile motionProfile;
            private ElapsedTime t;
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(!initialized){
                    initialPos = ticksToInches(encoder.getCurrentPosition());
                    reverse = !(targetHeight - initialPos >= 0);
                    motionProfile = new MotionProfile(Math.abs(targetHeight-initialPos), maxVelocity, maxAcceleration, maxDeceleration, reverse);
                    t = new ElapsedTime();
                    initialized = true;
                }
                checkLimit();

                double currentPosition = ticksToInches(encoder.getCurrentPosition());
                double ffPower = feedForward.calculate(motionProfile.getVelocity(t.seconds()), motionProfile.getAcceleration(t.seconds()));
                double pidPower = pid.calculate(initialPos + motionProfile.getPos(t.seconds()), currentPosition);
                motorPower = pidPower + ffPower ;
                if (Math.abs(targetHeight - currentPosition) < extensionThreshold){
                    extensionMotor.setPower(0);
                } else {
                    extensionMotor.setPower(motorPower);
                }
                packet.put("Current height (in)", currentPosition);
                packet.put("Target height (in)", targetHeight);
                packet.put("Motor velocity (in/s)", extensionMotor.getVelocity()/ticksPerInch);
                packet.put("Target velocity (in/s)", motionProfile.getVelocity(t.seconds()));
                packet.put("Feedforward power", ffPower);
                packet.put("PID power", pidPower);
                packet.put("Motor power", motorPower);

                return Math.abs(targetHeight - currentPosition) > extensionThreshold;
            }
        };
    }
    public Action manualControl(double power) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (limiter.isPressed()){
                    extensionMotor.setPower(0);
                    packet.put("Motor Power", power);
                } else {
                    extensionMotor.setPower(power);
                    packet.put("Motor Power", power);
                }
                return false;
            }
        };
    }
    public Action infiniteHold(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return true;
            }
        };
    }
}
