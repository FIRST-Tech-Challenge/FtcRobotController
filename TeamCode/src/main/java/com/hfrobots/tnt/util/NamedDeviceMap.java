/**
 Copyright (c) 2016 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.util;

import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Enables reflective use of hardware devices and iterating
 * them by type with their name (not part of the FTC SDK)
 */
@SuppressWarnings("unchecked")
public class NamedDeviceMap {
    public static class NamedDevice<T> {
        private final T device;

        private final String name;

        public NamedDevice(T device, String name) {
            this.device = device;
            this.name = name;
        }

        public T getDevice() {
            return device;
        }

        public String getName() {
            return name;
        }
    }

    private final HardwareMap hardwareMap;

    private final Map<Object, String> namesToHardwareMap = new HashMap();


    public NamedDeviceMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        buildDeviceToNameMapping(hardwareMap.dcMotorController);

        buildDeviceToNameMapping(hardwareMap.dcMotor);

        buildDeviceToNameMapping(hardwareMap.servoController);
        buildDeviceToNameMapping(hardwareMap.servo);
        buildDeviceToNameMapping(hardwareMap.crservo);

        buildDeviceToNameMapping(hardwareMap.legacyModule);
        buildDeviceToNameMapping(hardwareMap.touchSensorMultiplexer);

        buildDeviceToNameMapping(hardwareMap.deviceInterfaceModule);
        buildDeviceToNameMapping(hardwareMap.analogInput);
        buildDeviceToNameMapping(hardwareMap.digitalChannel);
        buildDeviceToNameMapping(hardwareMap.opticalDistanceSensor);
        buildDeviceToNameMapping(hardwareMap.touchSensor);
        buildDeviceToNameMapping(hardwareMap.pwmOutput);
        buildDeviceToNameMapping(hardwareMap.i2cDevice);
        buildDeviceToNameMapping(hardwareMap.i2cDeviceSynch);
        buildDeviceToNameMapping(hardwareMap.analogOutput);
        buildDeviceToNameMapping(hardwareMap.colorSensor);
        buildDeviceToNameMapping(hardwareMap.led);

        buildDeviceToNameMapping(hardwareMap.accelerationSensor);
        buildDeviceToNameMapping(hardwareMap.compassSensor);
        buildDeviceToNameMapping(hardwareMap.gyroSensor);
        buildDeviceToNameMapping(hardwareMap.irSeekerSensor);
        buildDeviceToNameMapping(hardwareMap.lightSensor);
        buildDeviceToNameMapping(hardwareMap.ultrasonicSensor);
        buildDeviceToNameMapping(hardwareMap.voltageSensor);

        buildSensorNameMappings(Rev2mDistanceSensor.class);
        buildSensorNameMappings(LynxEmbeddedIMU.class);
    }

    private void buildSensorNameMappings(Class<? extends HardwareDevice> classOrInterface) {
        for (HardwareDevice sensor : hardwareMap.getAll(classOrInterface)) {
            Set<String> names = hardwareMap.getNamesOf(sensor);
            for (String name : names) {
                namesToHardwareMap.put(sensor, name);
            }
        }
    }

    private void buildDeviceToNameMapping(HardwareMap.DeviceMapping deviceMapping) {
        Set<Map.Entry> entrySet = deviceMapping.entrySet();
        for (Map.Entry entry : entrySet) {
            namesToHardwareMap.put(entry.getValue(), entry.getKey().toString());
        }
    }
    public <T> List<NamedDevice<T>> getAll(Class<? extends T> classOrInterface) {
        List<T> nonNamedDevices = hardwareMap.getAll(classOrInterface);
        List<NamedDevice<T>> namedDevices = new ArrayList();

        for (T device : nonNamedDevices) {
            String name = namesToHardwareMap.get(device);
            namedDevices.add(new NamedDevice<T>(device, name));
        }

        return namedDevices;
    }
}
