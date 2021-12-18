package org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.src.Utills.Executable;
import org.firstinspires.ftc.teamcode.src.Utills.MiscUtills;
import org.firstinspires.ftc.teamcode.src.Utills.ThreadedSubsystemTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.RobotVoltageSensor;

import java.util.HashMap;

public class LinearSlide extends ThreadedSubsystemTemplate {

    private final DcMotor linearSlide;
    private static final Object lock = new Object();
    private final RobotVoltageSensor voltageSensor;
    private volatile int targetHeight;


    public LinearSlide(HardwareMap hardwareMap, String dcMotorName, RobotVoltageSensor voltSensor,
                       Executable<Boolean> _isOpmodeActive, Executable<Boolean> _isStopRequested) {
        super(_isOpmodeActive, _isStopRequested);
        linearSlide = hardwareMap.dcMotor.get(dcMotorName);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.voltageSensor = voltSensor;
    }

    public boolean isAtPosition() {
        return this.isAtPosition(80);
    }

    public boolean isAtPosition(int tolerence) {
        int currentPos = this.getEncoderCount();
        int target = this.linearSlide.getTargetPosition();
        int dist = Math.abs(currentPos - target);
        return dist < tolerence;
    }

    public int getTargetHeight() {
        return this.targetHeight;
    }

    public void setTargetHeight(int height) {
        targetHeight = height;
    }

    public void setTargetLevel(HeightLevel level) {
        synchronized (lock) {
            setTargetHeight(HeightLevel.EncoderCount.get(level));
        }
    }

    public void setMotorPower(double power) {
        linearSlide.setPower(power);
    }

    public int getEncoderCount() {
        return linearSlide.getCurrentPosition();
    }

    public void threadMain() {
        double power;// power = -(distance* C1 )^3 + (voltage * C2)
            double volts = voltageSensor.getVoltage();
        int encoderTicks = linearSlide.getCurrentPosition();
        int distanceFromPos = targetHeight - encoderTicks;
        final double C1 = 0.01; //Scales the sensitivity of the function, smaller value is lower sensativity
            final double C2 = 0.017; //The approximate power required to hold itself at the current height

            power = (Math.pow(((distanceFromPos * C1)), 3));
            power = power + (volts * C2);
            power = MiscUtills.boundNumber(power);

        linearSlide.setPower(power);
    }

    public void resetEncoder() {
        this.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public enum HeightLevel {
        Down,
        BottomLevel,
        MiddleLevel,
        TopLevel,
        GetOverObstacles;
        protected static final HashMap<HeightLevel, Integer> EncoderCount = new HashMap<HeightLevel, Integer>() {{
            put(HeightLevel.BottomLevel, -876);
            put(HeightLevel.MiddleLevel, -1763);
            put(HeightLevel.TopLevel, -2800);
            put(HeightLevel.GetOverObstacles, -675);
            put(HeightLevel.Down, 0);
        }};
    }
}
