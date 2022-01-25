package org.firstinspires.ftc.teamcode.src.robotAttachments.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.ThreadedSubsystemTemplate;

/**
 * This class executes a function if the distance sensor detects a value less than threshold
 */
public class TripWireDistanceSensor extends ThreadedSubsystemTemplate {
    private final DistanceSensor distanceSensor;
    private final double threshold;
    private final Executable<Void> callBack;

    /**
     * A constructor that instantiates the Executable Objects, they allow the thread to end with the OpMode
     *
     * @param hardwareMap     The Hardware map object to make the distance sensor from
     * @param distanceSensor  The name of the distance sensor to make
     * @param threshold       the threshold for function activation
     * @param callBack        A Executable object wrapped around a function to execute on call back
     * @param opModeIsActive  A Executable object wrapped around {@link LinearOpMode#opModeIsActive()}
     * @param isStopRequested A Executable object wrapped around {@link LinearOpMode#isStopRequested()}
     */
    public TripWireDistanceSensor(HardwareMap hardwareMap, String distanceSensor, double threshold, Executable<Void> callBack, Executable<Boolean> opModeIsActive, Executable<Boolean> isStopRequested) {
        super(opModeIsActive, isStopRequested);
        this.distanceSensor = (DistanceSensor) hardwareMap.get(distanceSensor);
        this.threshold = threshold;
        this.callBack = callBack;

    }

    /**
     * The function that monitors the distance sensor
     */
    @Override
    public void threadMain() {
        if (distanceSensor.getDistance(DistanceUnit.CM) < threshold) {
            this.callBack.call();
        }
    }

    @Override
    protected void onEnd() {
        distanceSensor.close();
    }
}
