package org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.src.Utills.Executable;
import org.firstinspires.ftc.teamcode.src.Utills.MiscUtills;
import org.firstinspires.ftc.teamcode.src.Utills.ThreadedSubsystemTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.RobotVoltageSensor;

import java.util.HashMap;

/**
 * A class to control linear slides
 */
public class LinearSlide extends ThreadedSubsystemTemplate {

    /**
     * A lock for thread synchronization
     */
    private static final Object lock = new Object();
    /**
     * The control motor
     */
    private final DcMotor linearSlide;
    /**
     * Voltage sensor
     */
    private final RobotVoltageSensor voltageSensor;

    /**
     * The current position the Linear Slide is going to
     */
    private volatile int targetHeight;


    /**
     * Constructs the slide based on hardware map
     *
     * @param hardwareMap      Hardware Map Object from OpMode
     * @param dcMotorName      Name of the control motor
     * @param voltSensor       A Voltage Sensor Object
     * @param _isStopRequested A Executable object wrapped around OpMode.isStopRequested()
     * @param _isOpmodeActive  A Executable object wrapped around OpMode.opModeIsActive()
     */
    public LinearSlide(HardwareMap hardwareMap, String dcMotorName, RobotVoltageSensor voltSensor,
                       Executable<Boolean> _isOpmodeActive, Executable<Boolean> _isStopRequested) {
        super(_isOpmodeActive, _isStopRequested);
        linearSlide = hardwareMap.dcMotor.get(dcMotorName);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.voltageSensor = voltSensor;
    }

    /**
     * Determines if the linear slide is close to it's given position
     *
     * @return True if it is close enough, false otherwise
     */
    public boolean isAtPosition() {
        return this.isAtPosition(80);
    }

    /**
     * Determines if the linear slide is close to it's given position
     *
     * @param tolerance How close is good enough (in ticks)
     * @return True if it is close enough, false otherwise
     */
    public boolean isAtPosition(int tolerance) {
        int currentPos = this.getEncoderCount();
        int target = this.linearSlide.getTargetPosition();
        int dist = Math.abs(currentPos - target);
        return dist < tolerance;
    }

    /**
     * Getter for the target height
     *
     * @return Returns the position the Slide is set to go to
     */
    public int getTargetHeight() {
        return this.targetHeight;
    }

    /**
     * A setter for the target height
     *
     * @param height The height that the slide should go to
     */
    public void setTargetHeight(int height) {
        targetHeight = height;
    }

    /**
     * A setter for the target height
     *
     * @param level A enum of height level to go to
     */
    public void setTargetLevel(HeightLevel level) {
        synchronized (lock) {
            setTargetHeight(HeightLevel.EncoderCount.get(level));
        }
    }

    /**
     * A setter for the motor power
     *
     * @param power The power to set to
     */
    public void setMotorPower(double power) {
        linearSlide.setPower(MiscUtills.boundNumber(power));
    }

    /**
     * A getter for the slide's current position
     *
     * @return Current position in ticks
     */
    public int getEncoderCount() {
        return linearSlide.getCurrentPosition();
    }

    /**
     * The main function for the Slide Thread
     */
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

    /**
     * Resets the Linear Slide Encoder
     */
    public void resetEncoder() {
        this.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    /**
     * A Enum for Height Levels
     */
    public enum HeightLevel {
        Down,
        BottomLevel,
        MiddleLevel,
        TopLevel,
        GetOverObstacles;
        /**
         * Key is the Height Level, Value is the position to go to in ticks
         */
        protected static final HashMap<HeightLevel, Integer> EncoderCount = new HashMap<HeightLevel, Integer>() {{
            put(HeightLevel.BottomLevel, -876);
            put(HeightLevel.MiddleLevel, -1763);
            put(HeightLevel.TopLevel, -2800);
            put(HeightLevel.GetOverObstacles, -675);
            put(HeightLevel.Down, 0);
        }};
    }
}
