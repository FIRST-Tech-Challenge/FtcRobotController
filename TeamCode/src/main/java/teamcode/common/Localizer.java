package teamcode.common;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.io.File;
import java.util.concurrent.atomic.AtomicBoolean;

import static java.lang.Math.*;

public class Localizer extends Thread {
    //TODO before reading this file please note the static import of the math class,
    // odds are if you see a math function it is from that and not a constatnt/method I created
    //https://docs.google.com/document/d/1JQuU2M--rVFEa9ApKkOcc0CalsBxKjH-l0PQLRW7F3c/edit?usp=sharing proof behind the math

    private static final double TICKS_PER_REV = 8192;
    private static final double WHEEL_RADIUS = 1.421; //1.181 for 60 mm, 1.417 for 72mm
    private static final double GEAR_RATIO = 1;
    private static final double CHASSIS_LENGTH = 12.4; //new bot + 2.83465
    private static final double ODO_XY_DISTANCE = 2.5;


    File loggingFile = AppUtil.getInstance().getSettingsFile("MotionLogging.txt");
    String loggingString;
    //-2.641358450698 - (-2.641358450698 * 1.2)

    /**
     * Position stored in ticks. When exposed to external classes, it is represented in inches.
     */




    // set to true when thread is requested to shutdown
    private AtomicBoolean stop = new AtomicBoolean(false);
    // sensors run at 300 Hz
    // this is the length of that interval in nanoseconds
    private long runInterval = (long)Math.round(1.0/300.0 * 1000000.0);
    //

    private long elapsedTime, startingTime;
    private RobotPositionStateUpdater state;
    private final ExpansionHubMotor leftVertical, rightVertical, horizontal;
    private final ExpansionHubEx hub1;
    private RevBulkData data1, data2;
    private final BNO055IMU imu;
    private double previousOuterArcLength = 0;
    private double previousInnerArcLength = 0;
    private double previousHorizontalArcLength = 0;
    /**
     * @param position in inches
     * @param globalRads in radians
     */
    public Localizer(HardwareMap hardwareMap, Vector2D position, double globalRads) {
        hub1 = hardwareMap.get(ExpansionHubEx.class,"Control Hub");
        loggingString = "";
        //hub2 = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");
        // initialize hardware
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        leftVertical = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.LEFT_VERTICAL_ODOMETER_NAME);
        rightVertical = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.RIGHT_VERTICAL_ODOMETER_NAME);
        horizontal = (ExpansionHubMotor)hardwareMap.dcMotor.get(Constants.HORIZONTAL_ODOMETER_NAME);
        // setup initial position;
        previousHorizontalArcLength = 0;
        previousInnerArcLength = 0;
        previousOuterArcLength = 0;
        startingTime = System.currentTimeMillis();
        state = new RobotPositionStateUpdater(position, new Vector2D(0,0), globalRads, 0);
        resetEncoders();

    }
    public void stopThread() {
        this.stop.set(true);
    }
    @Override
    public void run() {
        // make sure we reset our accounting of start times
        state.resetUpdateTime();
        // max speed 300 Hz)
        while (!stop.get()) {
            long nanos = System.nanoTime();
            update();
            long runtime = System.nanoTime() - nanos;
            if (runtime > runInterval) {
                // this is very bad
                // todo break here.
                runtime=runInterval;
            }
            try {
                // cast should be fine here since we're likely dealing with only
                // a few milliseconds
                sleep(0,(int)(runInterval - runtime));
            } catch (InterruptedException e) {
                // this probably isn't bad.
                e.printStackTrace();
            }
        }
    }

    private void resetEncoders() {
        leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVertical.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetOdometersTravelling(){
        leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        previousHorizontalArcLength = 0;
        previousInnerArcLength = 0;
        previousOuterArcLength = 0;
    }

    public RobotPositionStateUpdater.RobotPositionState getCurrentState() {
        return state.getCurrentState();
    }

    //see top of class for formalized proof of the math
    private synchronized void update() {

        // read sensor data
        data1 = hub1.getBulkInputData();
        double innerArcLength = encoderTicksToInches(data1.getMotorCurrentPosition(leftVertical));
        // encoder orientation is the same, which means they generate opposite rotation signals
        double outerArcLength = - encoderTicksToInches(data1.getMotorCurrentPosition(rightVertical));
        double horizontalArcLength = -encoderTicksToInches(data1.getMotorCurrentPosition(horizontal));

        double leftVerticalVelocity = encoderTicksToInches(data1.getMotorVelocity(leftVertical));
        double rightVerticalVelocity = encoderTicksToInches(data1.getMotorVelocity(rightVertical));
        double horizontalVelocity = -encoderTicksToInches(data1.getMotorVelocity(horizontal));

        // calculate positions
        double deltaInnerArcLength = innerArcLength - previousInnerArcLength;
        double deltaOuterArcLength = outerArcLength - previousOuterArcLength;
        double deltaHorizontalArcLength = horizontalArcLength - previousHorizontalArcLength;

        double arcLength = (deltaInnerArcLength + deltaOuterArcLength) / 2.0;
        double deltaVerticalDiff = (deltaInnerArcLength - deltaOuterArcLength) / 2.0;

//(deltaOuterArcLength - deltaInnerArcLength)
        // CHASSIS_LENGTH is the diamater of the circle.
        // phi is arclength divided by radius for small phi
        double phi =  (2.0 * arcLength) / (CHASSIS_LENGTH);
        double hypotenuse;
        // When phi is small, the full formula is numerically unstable.
        // for small phi, sin(phi) = phi and cos(phi) = 1
        // thus small phi, hypotense = arcLength
        if(abs(phi) < 0.0001){
            hypotenuse = arcLength;
        }else{
            hypotenuse = (arcLength * sin(phi)) / (phi * cos(phi / 2.0));
        }

        double horizontalDelta = deltaHorizontalArcLength - (phi * ODO_XY_DISTANCE);
        double verticalDelta = hypotenuse * cos(phi/2.0)+ deltaVerticalDiff;

        // calculate velocities
        // a difference in velocity will be due to rotation.
        // however since both encoders count this difference, this is double counted
        // So arc length of rotation divided by the radius gives us the rotational velocity
        // the factors of two cancel!
        double omega = (leftVerticalVelocity - rightVerticalVelocity)/CHASSIS_LENGTH;
        double deltaVy = (leftVerticalVelocity + rightVerticalVelocity)/2.0;
        double deltaVx = horizontalVelocity;
        //Debug.log(horizontalDelta);
        state.updateDelta(horizontalDelta, verticalDelta, phi, deltaVx, deltaVy, omega);
        previousInnerArcLength = innerArcLength;
        previousOuterArcLength = outerArcLength;
        previousHorizontalArcLength = horizontalArcLength;
        elapsedTime = System.currentTimeMillis() - startingTime;
        loggingString +=  elapsedTime + "," + state.getCurrentState().loggingToString() + "\n";
        ReadWriteFile.writeFile(loggingFile, loggingString);
    }


    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public static int inchesToEncoderTicks(double inches){
        return (int)((TICKS_PER_REV / (WHEEL_RADIUS * 2 * PI * GEAR_RATIO)) * inches);
    }

    public int getLeftVerticalOdometerPosition(){
        return -leftVertical.getCurrentPosition();
    }

    public int getRightVerticalOdometerPosition(){
        return -rightVertical.getCurrentPosition();
    }

    public int getHorizontalOdometerPosition(){
        return -horizontal.getCurrentPosition();
    }
}
