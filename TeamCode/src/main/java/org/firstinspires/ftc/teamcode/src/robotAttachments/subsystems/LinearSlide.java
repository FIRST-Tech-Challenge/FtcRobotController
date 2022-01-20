package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtills;
import org.firstinspires.ftc.teamcode.src.utills.ThreadedSubsystemTemplate;

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
     * Reverses the motor
     */
    public void reverseMotor() {
        if (this.linearSlide.getDirection() == DcMotorSimple.Direction.FORWARD) {
            this.linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            this.linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    /**
     * Constructs the slide based on hardware map
     *
     * @param hardwareMap     Hardware Map Object from OpMode
     * @param dcMotorName     Name of the control motor
     * @param voltSensor      A Voltage Sensor Object
     * @param isStopRequested A Executable object wrapped around {@link LinearOpMode#isStopRequested()}
     * @param opModeIsActive  A Executable object wrapped around {@link LinearOpMode#opModeIsActive()}
     */
    public LinearSlide(HardwareMap hardwareMap, String dcMotorName, RobotVoltageSensor voltSensor,
                       Executable<Boolean> opModeIsActive, Executable<Boolean> isStopRequested) {
        super(opModeIsActive, isStopRequested);
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
        int dist = Math.abs(this.getEncoderCount() - this.linearSlide.getTargetPosition());
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
        synchronized (lock) {
            targetHeight = height;
        }
    }

    /**
     * A setter for the target height
     *
     * @param level A enum of height level to go to
     */
    public void setTargetLevel(HeightLevel level) {
        Integer height = HeightLevel.EncoderCount.get(level);
        assert height != null;
        setTargetHeight(height);
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

        final double C1 = 0.015; //Scales the sensitivity of the function, smaller value is lower sensativity
        final double C2 = 0.1; //The approximate power required to hold itself at the current height

        power = (Math.pow(((distanceFromPos * C1)), 3));
        power = power + (C2) * (volts * (1 / 12.0));
        power = MiscUtills.boundNumber(power);
        //if (distanceFromPos <= 0){power = 0;}

        linearSlide.setPower(power);
    }

    /**
     * Resets the Linear Slide Encoder
     */
    public void resetEncoder() {
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    protected void onEnd() {
        linearSlide.setPower(0);
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
            put(HeightLevel.BottomLevel, 0);
            put(HeightLevel.MiddleLevel, 233);
            put(HeightLevel.TopLevel, 584);
            put(HeightLevel.GetOverObstacles, 0);
            put(HeightLevel.Down, 0);
        }};
    }
}
