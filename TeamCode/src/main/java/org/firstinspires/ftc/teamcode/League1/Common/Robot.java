package org.firstinspires.ftc.teamcode.League1.Common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.concurrent.atomic.AtomicBoolean;

public class Robot extends Thread{




    private volatile LynxModule.BulkData ehData, chData;
    private volatile NormalizedRGBA grabberRGBA, driveRGBA;
    private volatile Orientation imuAngle;

    private LynxModule ch, eh;
    private ExpansionHubEx ehub, chub;
    private ColorRangeSensor grabberColor, driveColor;
    private BNO055IMU imu;

    AtomicBoolean shouldUpdate;
    AtomicBoolean controlProcessRunning;
    private final long runtime = 20;

    private final float COLOR_GAIN = 3;




    //ColorRangeSensor color;
    public synchronized void update(){
        chData = ch.getBulkData();
        ehData = eh.getBulkData();
        imuAngle = imu.getAngularOrientation();

        grabberRGBA = grabberColor.getNormalizedColors();
        driveRGBA = driveColor.getNormalizedColors();

        eh.clearBulkCache();
        ch.clearBulkCache();
    }


    @Override
    public void run() {
        while(controlProcessRunning.get()){
            if(shouldUpdate.get()) {
                long loopStart = System.currentTimeMillis();
                update();
                long loopDelta = System.currentTimeMillis() - loopStart;
                if (loopDelta <= runtime) {
                    try {
                        Thread.currentThread().sleep(runtime - loopDelta);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                } else {
                    //memory leaks are bad
                }
            }
        }
    }

    //Control Constructor
    public Robot(HardwareMap hardwareMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        eh = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        ch = hardwareMap.get(LynxModule.class, "Control Hub");

        ehub  = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        chub  = hardwareMap.get(ExpansionHubEx.class, "Control Hub");

        ehub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FAST_400K);
        chub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FAST_400K);

        grabberColor = hardwareMap.get(ColorRangeSensor.class, "Color");
        grabberColor.setGain(COLOR_GAIN);

        driveColor = hardwareMap.get(ColorRangeSensor.class, "Color");
        driveColor.setGain(COLOR_GAIN);


        eh.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        ch.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        eh.clearBulkCache();
        ch.clearBulkCache();

        shouldUpdate = new AtomicBoolean(true);
        controlProcessRunning = new AtomicBoolean(true);
    }

    public NormalizedRGBA getRGBA(boolean grabber){
        if(grabber){
            return grabberRGBA;
        }else{
            return driveRGBA;
        }
    }


    public LynxModule.BulkData getBulkPacket(boolean chub){
        if(chub){
            return chData;
        }else{
            return ehData;
        }
    }

    /*

    public double getDirection() {
        return imuAngle.firstAngle;
    }

     */

    public double getDirection(){
        return imu.getAngularOrientation().firstAngle;
    }

    public void setShouldUpdate(boolean val){
        shouldUpdate.set(val);
    }

    public void stopThread() {
        shouldUpdate.set(false);
        imu.close();
        eh.clearBulkCache();
        ch.clearBulkCache();

    }
}