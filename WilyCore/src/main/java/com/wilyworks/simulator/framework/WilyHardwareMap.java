package com.wilyworks.simulator.framework;


import android.content.Context;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.SerialNumber;
import com.wilyworks.common.WilyWorks;
import com.wilyworks.simulator.WilyCore;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;

import kotlin.coroutines.Continuation;

/**
 * Wily Works device subclass implementation.
 */
class WilyHardwareDevice implements HardwareDevice {
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "";
    }

    @Override
    public String getConnectionInfo() {
        return "";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}

/**
 * Wily Works simulated IMU implementation.
 */
class WilyIMU extends WilyHardwareDevice implements IMU {
    double startYaw;
    @Override
    public boolean initialize(Parameters parameters) {
        resetYaw();
        return true;
    }

    @Override
    public void resetYaw() {
        startYaw = WilyWorks.getPose().heading.log();
    }

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return new YawPitchRollAngles(
                AngleUnit.RADIANS,
                WilyWorks.getPose().heading.log() - startYaw,
                0,
                0,
                0);
    }

    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        return new Orientation();
    }

    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        return new Quaternion();
    }

    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        return new AngularVelocity(
                angleUnit,
                (float) WilyWorks.getPoseVelocity().angVel, // ### transformedAngularVelocityVector.get(0),
                0, // ### transformedAngularVelocityVector.get(1),
                0, // ### transformedAngularVelocityVector.get(2),
                0); // ### rawAngularVelocity.acquisitionTime);
    }
}

/**
 * Wily Works voltage sensor implementation.
 */
class WilyVoltageSensor extends WilyHardwareDevice implements VoltageSensor {
    @Override
    public double getVoltage() {
        return 13.0;
    }

    @Override
    public String getDeviceName() {
        return "Voltage Sensor";
    }
}

/**
 * Wily Works distance sensor implementation.
 */
class WilyDistanceSensor extends WilyHardwareDevice implements DistanceSensor {
    @Override
    public double getDistance(DistanceUnit unit) { return DistanceUnit.infinity; }
}

/**
 * Wily Works named webcam implementation.
 */
class WilyWebcam extends WilyHardwareDevice implements WebcamName {
    WilyWorks.Config.Camera wilyCamera;

    WilyWebcam(String deviceName) {
        for (WilyWorks.Config.Camera camera: WilyCore.config.cameras) {
            if (camera.name.equals(deviceName)) {
                wilyCamera = camera;
            }
        }
        if (wilyCamera == null) {
            System.out.printf("WilyWorks: Couldn't find configuration data for camera '%s'", deviceName);
        }
    }

    @Override
    public boolean isWebcam() {
        return true;
    }

    @Override
    public boolean isCameraDirection() {
        return false;
    }

    @Override
    public boolean isSwitchable() {
        return false;
    }

    @Override
    public boolean isUnknown() {
        return false;
    }

    @Override
    public void asyncRequestCameraPermission(Context context, Deadline deadline, Continuation<? extends Consumer<Boolean>> continuation) {

    }

    @Override
    public boolean requestCameraPermission(Deadline deadline) {
        return false;
    }

    @Override
    public CameraCharacteristics getCameraCharacteristics() {
        return null;
    }

    @Override
    public SerialNumber getSerialNumber() {
        return null;
    }

    @Nullable
    @Override
    public String getUsbDeviceNameIfAttached() {
        return null;
    }

    @Override
    public boolean isAttached() {
        return false;
    }
}

/**
 * Wily Works Servo implementation.
 */
class WilyServo extends WilyHardwareDevice implements Servo {

    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPosition(double position) {

    }

    @Override
    public double getPosition() {
        return 0;
    }

    @Override
    public void scaleRange(double min, double max) {

    }
}

/**
 * Wily Works CRServo implementation.
 */
class WilyCRServo extends WilyHardwareDevice implements CRServo {

    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPower(double power) {

    }

    @Override
    public double getPower() {
        return 0;
    }
}

/**
 * Wily Works DcMotorEx implementation.
 */
class WilyDcMotorEx extends WilyHardwareDevice implements DcMotorEx {

    @Override
    public DcMotorController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return null;
    }

    @Override
    public boolean getPowerFloat() {
        return false;
    }

    @Override
    public void setTargetPosition(int position) {

    }

    @Override
    public int getTargetPosition() {
        return 0;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public int getCurrentPosition() {
        return 0;
    }

    @Override
    public void setMode(RunMode mode) {

    }

    @Override
    public RunMode getMode() {
        return null;
    }

    @Override
    public void setMotorEnable() {

    }

    @Override
    public void setMotorDisable() {

    }

    @Override
    public boolean isMotorEnabled() {
        return false;
    }

    @Override
    public void setVelocity(double angularRate) {

    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {

    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return 0;
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {

    }

    @Override
    public void setPositionPIDFCoefficients(double p) {

    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {

    }

    @Override
    public int getTargetPositionTolerance() {
        return 0;
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return 0;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return 0;
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {

    }

    @Override
    public boolean isOverCurrent() {
        return false;
    }

    @Override
    public void setDirection(Direction direction) { }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPower(double power) { }

    @Override
    public double getPower() {
        return 0;
    }
}

/**
 * Wily Works DigitalChannel implementation.
 */
class WilyDigitalChannel extends WilyHardwareDevice implements DigitalChannel {

    @Override
    public Mode getMode() { return null; }

    @Override
    public void setMode(Mode mode) {}

    @Override
    public boolean getState() { return false; }

    @Override
    public void setState(boolean state) {}
}

/**
 * Wily Works hardware map.
 */
public class WilyHardwareMap implements Iterable<HardwareDevice> {

    public DeviceMapping<VoltageSensor>         voltageSensor         = new DeviceMapping<VoltageSensor>(VoltageSensor.class);
    public DeviceMapping<DcMotor>               dcMotor               = new DeviceMapping<DcMotor>(DcMotor.class);
    public DeviceMapping<DistanceSensor>        distanceSensor        = new DeviceMapping<DistanceSensor>(DistanceSensor.class);
    public DeviceMapping<WebcamName>            webcamName            = new DeviceMapping<WebcamName>(WebcamName.class);
    public DeviceMapping<Servo>                 servo                 = new DeviceMapping<>(Servo.class);
    public DeviceMapping<CRServo>               crservo               = new DeviceMapping<>(CRServo.class);
    public DeviceMapping<DigitalChannel>        digitalChannel        = new DeviceMapping<>(DigitalChannel.class);

    protected Map<String, List<HardwareDevice>> allDevicesMap         = new HashMap<>();
    protected List<HardwareDevice>              allDevicesList        = new ArrayList<>();

    public WilyHardwareMap() {
        put("voltage_sensor", VoltageSensor.class);
    }

    public final Context appContext = new Context();
    protected final Object lock = new Object();

    public <T> List<T> getAll(Class<? extends T> classOrInterface) {
        List<T> result = new LinkedList<T>();
        return result;
    }

    private synchronized <T> T tryGet(Class<? extends T> classOrInterface, String deviceName){
        List<HardwareDevice> list = allDevicesMap.get(deviceName.trim());
        if (list != null) {
            for (HardwareDevice device : list){
                if(classOrInterface.isInstance(device)) return classOrInterface.cast(device);
            }
        }
        return null;
    }

    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        if (classOrInterface.equals(IMU.class)) {
            return (T) new WilyIMU();
        }
        T result = tryGet(classOrInterface, deviceName);
        if (result == null) {
            // Wily Works behavior is that we automatically add the device if it's not found:
            put(deviceName, classOrInterface);
            result = tryGet(classOrInterface, deviceName);
        }
        return result;
    }

    @Deprecated
    public HardwareDevice get(String deviceName) {
        List<HardwareDevice> list = allDevicesMap.get(deviceName.trim());
        if (list != null) {
            return list.get(0);
        }

        throw new IllegalArgumentException("Use the typed version of get(), e.g. get(DcMotorEx.class, \"leftMotor\")");
    }

    // Wily Works way to add devices to the hardware map:
    public synchronized void put(String deviceName, Class klass) {
        deviceName = deviceName.trim();
        List<HardwareDevice> list = allDevicesMap.get(deviceName);
        if (list == null) {
            list = new ArrayList<>();
            allDevicesMap.put(deviceName, list);
        }
        HardwareDevice device;
        if (CRServo.class.isAssignableFrom(klass)) {
            device = new WilyCRServo();
            crservo.put(deviceName, (CRServo) device);
        } else if (Servo.class.isAssignableFrom(klass)) {
            device = new WilyServo();
            servo.put(deviceName, (Servo) device);
        } else if (DcMotor.class.isAssignableFrom(klass)) {
            device = new WilyDcMotorEx();
            dcMotor.put(deviceName, (DcMotor) device);
        } else if (VoltageSensor.class.isAssignableFrom(klass)) {
            device = new WilyVoltageSensor();
            voltageSensor.put(deviceName, (VoltageSensor) device);
        } else if (DistanceSensor.class.isAssignableFrom(klass)) {
            device = new WilyDistanceSensor();
            distanceSensor.put(deviceName, (DistanceSensor) device);
        } else if (WebcamName.class.isAssignableFrom(klass)) {
            device = new WilyWebcam(deviceName);
            webcamName.put(deviceName, (WebcamName) device);
        } else if (DigitalChannel.class.isAssignableFrom(klass)) {
            device = new WilyDigitalChannel();
            digitalChannel.put(deviceName, (DigitalChannel) device);
        } else {
            throw new IllegalArgumentException("Unexpected device type for HardwareMap");
        }
        list.add(device);
        allDevicesList.add(device);
    }

    private void initializeDeviceIfNecessary(HardwareDevice device) {
    }

    private void initializeMultipleDevicesIfNecessary(Iterable<? extends HardwareDevice> devices) {
        for (HardwareDevice device: devices) {
            initializeDeviceIfNecessary(device);
        }
    }

    public SortedSet<String> getAllNames(Class<? extends HardwareDevice> classOrInterface) {
        SortedSet<String> result = new TreeSet<>();
        result.add("voltage_sensor");
        return result;
    }

    @Override
    public @NonNull Iterator<HardwareDevice> iterator() {
        return new ArrayList<HardwareDevice>(allDevicesList).iterator();
    }

    // A DeviceMapping contains a sub-collection of the devices registered in a HardwareMap
    // comprised of all devices of a particular device type.
    public class DeviceMapping<DEVICE_TYPE extends HardwareDevice> implements Iterable<DEVICE_TYPE> {
        private final Map<String, DEVICE_TYPE> map = new HashMap<String, DEVICE_TYPE>();
        private final Class<DEVICE_TYPE> deviceTypeClass;

        public DeviceMapping(Class<DEVICE_TYPE> deviceTypeClass) {
            this.deviceTypeClass = deviceTypeClass;
        }

        public Class<DEVICE_TYPE> getDeviceTypeClass() {
            return this.deviceTypeClass;
        }

        public DEVICE_TYPE cast(Object obj) {
            return this.deviceTypeClass.cast(obj);
        }

        public DEVICE_TYPE get(String deviceName) {
            synchronized (lock) {
                deviceName = deviceName.trim();
                DEVICE_TYPE device = map.get(deviceName);
                if (device == null) {
                    String msg = String.format("Unable to find a hardware device with the name \"%s\"", deviceName);
                    throw new IllegalArgumentException(msg);
                }
                initializeDeviceIfNecessary(device);
                return device;
            }
        }

        public void put(String deviceName, DEVICE_TYPE device) {
            map.put(deviceName.trim(), device);
        }

        public void putLocal(String deviceName, DEVICE_TYPE device) {
            synchronized (lock) {
            }
        }

        public boolean contains(String deviceName) {
            synchronized (lock) {
                deviceName = deviceName.trim();
                return map.containsKey(deviceName);
            }
        }

        public boolean remove(String deviceName) {
            return remove(null, deviceName);
        }

        public boolean remove(@Nullable SerialNumber serialNumber, String deviceName) {
            synchronized (lock) {
                return false;
            }
        }

        @Override public @NonNull Iterator<DEVICE_TYPE> iterator() {
            synchronized (lock) {
                initializeMultipleDevicesIfNecessary(map.values());
                return new ArrayList<>(map.values()).iterator();
            }
        }

        public Set<Map.Entry<String, DEVICE_TYPE>> entrySet() {
            synchronized (lock) {
                initializeMultipleDevicesIfNecessary(map.values());
                return new HashSet<>(map.entrySet());
            }
        }

        public int size() {
            synchronized (lock) {
                return map.size();
            }
        }
    }
}
