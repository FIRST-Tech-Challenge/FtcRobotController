package org.firstinspires.ftc.teamcode.Mechanisms.Lift;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.FeedForward;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Encoder;
import org.firstinspires.ftc.teamcode.Hardware.Actuators.DcMotorAdvanced;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Planners.MotionProfile;

@Config
public class Lift {
    HardwareMap hardwareMap;
    FeedForward feedForward;
    PID pid;
    double motorPower;
    public int currentPosition;
    public DcMotorAdvanced liftMotorLeft;
    public DcMotorAdvanced liftMotorRight;
    Encoder encoder;
    TouchSensor limiter;
    public static double kA=0;
    public static double kV=0;
    public static double kS=0.1;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    double spoolRadius =  0.702; // [in]
    int ticksPerRev = 1024;
    public static double maxAcceleration = 50.0;
    public static double maxDeceleration = 50.0;
    public static double maxVelocity = 60;
    boolean reverse;
    public static double maxVoltage = 12.5;

    // FIGURE OUT LIMIT SWITCHES/TOUCH SENSORS THEY SHOULD BE HERE


    public Lift(HardwareMap hardwareMap, Battery battery){
        this.hardwareMap = hardwareMap;

        this.liftMotorLeft = new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, "liftMotorLeft"), battery, maxVoltage);
        this.liftMotorRight = new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, "liftMotorRight"), battery, maxVoltage);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, "liftMotorRight"));
        this.liftMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.liftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.liftMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        this.liftMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.liftMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        this.liftMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.liftMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.liftMotorLeft.setPower(0);
        this.liftMotorRight.setPower(0);

        this.currentPosition = 0;
        this.feedForward = new FeedForward(kV, kA, kS);
        this.pid = new PID(kP, kI, kD);
        this.limiter = hardwareMap.get(TouchSensor.class, "liftTouch");
    }

    // FUNCTION TO RESET CURRENT ENCODER POSITION WHEN LIMIT SWITCHES ARE HIT
    private void checkLimit(){
        if (limiter.isPressed()){
            encoder.reset();
        }
    }

    // You should not need this. Do your control with respect to height in inches
    private int inchesToTicks(double inches) {
        return (int) ((inches / (2 * Math.PI * spoolRadius)) * ticksPerRev);
    }

    // See above but rename the parameter to targetHeight
    public Action moveToHeight(double targetHeight) {

        // You have edge cases. Motion profiles should work with a DISTANCE not target position.
        // You may run into a case where your current position is not zero and your target is
        // the basket height. You should get the sign of the difference between target and
        // current, then use that to set the reverse boolean in the motion profile. Then use the
        // absolute value of the difference to set the distance.

        reverse = !(targetHeight - currentPosition >= 0);

        MotionProfile motionProfile = new MotionProfile(Math.abs(targetHeight-currentPosition), maxVelocity, maxAcceleration, maxDeceleration, reverse);

        // When you call getPos from the motion profile, you're getting a distance NOT a target. How 
        // can you solve this? Hint: add two things.
        
        ElapsedTime t = new ElapsedTime();
        int initialPos = currentPosition;
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                checkLimit();
                int currentPosition = encoder.getCurrentPosition();

                // Edge case where you're moving down and not up. Maybe don't need FF when moving down?
                double ffPower = feedForward.calculate(motionProfile.getVelocity(t.seconds()), motionProfile.getAcceleration(t.seconds()));
                if (reverse) {
                    double pidPower = pid.calculate(initialPos + motionProfile.getPos(t.seconds()), currentPosition);
                    motorPower = pidPower + ffPower;
                } else {
                    motorPower = kS + ffPower;
                }
                liftMotorLeft.setPower(motorPower);
                liftMotorRight.setPower(motorPower);
                return Math.abs(targetHeight - currentPosition) < 0.1;
            }
        };
    }
    public Action manualControl(double power) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftMotorLeft.setPower(power-kS);
                liftMotorRight.setPower(power-kS);
                return false;
            }
        };
    }

    // Write another action that uses the joystick to move up and down.
}