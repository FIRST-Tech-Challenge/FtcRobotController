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

    private static final double ticksPerRevolution = 145.1;
    private static final double spoolRadius = 0.55 / 2; //in inches
    public RobotVoltageSensor voltageSensor;
    public volatile int chosenPosition;


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

    public void setTargetHeight(int height) {
        chosenPosition = height;
    }

    public void setTargetLevel(HeightLevel level) {
        setTargetHeight(HeightLevel.EncoderCount.get(level));
    }

    public void setMotorPower(double power) {
        linearSlide.setPower(power);
    }

    public int getEncoderCount() {
        return linearSlide.getCurrentPosition();
    }

    public void threadMain() {
        double power;// power = -(distance* C1 )^3 + (voltage * C2)
        {
            double volts = voltageSensor.getVoltage();
            int encoderTicks = linearSlide.getCurrentPosition();
            int distanceFromPos = chosenPosition - encoderTicks;
            final double C1 = 0.01; //Scales the sensitivity of the function, smaller value is lower sensativity
            final double C2 = 0.017; //The approximate power required to hold itself at the current height

            power = (Math.pow(((distanceFromPos * C1)), 3));
            power = power + (volts * C2);
            power = MiscUtills.boundNumber(power);
        }
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
            put(HeightLevel.BottomLevel, -1081);
            put(HeightLevel.MiddleLevel, 2560);
            put(HeightLevel.TopLevel, -4000);
            put(HeightLevel.GetOverObstacles, -1000);
            put(HeightLevel.Down, 0);
        }};
    }
}
