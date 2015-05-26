/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller;

import android.app.Activity;
import android.content.Context;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.Toast;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.modernrobotics.ModernRoboticsDeviceManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.DeviceManager.DeviceType;
import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.LegacyModuleControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.util.SerialNumber;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class AutoConfigureActivity extends Activity {

  private Context context;
  private Button configureButton;
  private DeviceManager deviceManager;
  protected Map<SerialNumber, DeviceManager.DeviceType> scannedDevices = new HashMap<SerialNumber, DeviceManager.DeviceType>();
  protected Set<Map.Entry<SerialNumber, DeviceManager.DeviceType>> entries = new HashSet<Map.Entry<SerialNumber, DeviceManager.DeviceType>>();
  private Map<SerialNumber, ControllerConfiguration> deviceControllers = new HashMap<SerialNumber, ControllerConfiguration>();

  private Thread t;
  private Utility utility;


  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    context = this;
    setContentView(R.layout.activity_autoconfigure);

    utility = new Utility(this);
    configureButton = (Button) findViewById(R.id.configureButton);

    try{
      deviceManager = new ModernRoboticsDeviceManager(context, null);
      //deviceManager = new MockDeviceManager(context, null);
    }catch (RobotCoreException e){
      utility.complainToast("Failed to open the Device Manager", context);
      DbgLog.error("Failed to open deviceManager: " + e.toString());
      DbgLog.logStacktrace(e);
    }
  }

  @Override
  protected void onStart() {
    super.onStart();

    utility.updateHeader(Utility.AUTOCONFIGURE_FILENAME, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
    if (utility.getFilenameFromPrefs(R.string.pref_hardware_config_filename, Utility.NO_FILE).equalsIgnoreCase(Utility.AUTOCONFIGURE_FILENAME)){
      warnAlreadyConfigured();
    } else {
      warnIfNoDevices();
    }
    configureButton.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View view) {

        t = new Thread(new Runnable() {
          @Override
          public void run() {
            try {
              DbgLog.msg("Scanning USB bus");
              scannedDevices = deviceManager.scanForUsbDevices();
            } catch (RobotCoreException e) {
              DbgLog.error("Device scan failed");
            }

            runOnUiThread(new Runnable() {
              @Override
              public void run() {
                utility.resetCount();
                if (scannedDevices.size() == 0) {
                  utility.saveToPreferences(Utility.NO_FILE, R.string.pref_hardware_config_filename);
                  utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
                  warnIfNoDevices();
                }
                entries = scannedDevices.entrySet();
                deviceControllers = new HashMap<SerialNumber, ControllerConfiguration>();

                utility.createLists(entries, deviceControllers);
                boolean foundRightDevices = buildK9();
                if (foundRightDevices) {
                  writeFile();
                } else {
                  utility.saveToPreferences(Utility.NO_FILE, R.string.pref_hardware_config_filename);
                  utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
                  warnWrongDevices();
                }
              }
            });
          }
        });
        t.start();
      }
    });
  }

  private void writeFile(){
    utility.writeXML(deviceControllers);
    try {
      utility.writeToFile(Utility.AUTOCONFIGURE_FILENAME+Utility.FILE_EXT);
      utility.saveToPreferences(Utility.AUTOCONFIGURE_FILENAME, R.string.pref_hardware_config_filename);
      utility.updateHeader(Utility.AUTOCONFIGURE_FILENAME, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
      Toast.makeText(context, "AutoConfigure Successful", Toast.LENGTH_SHORT).show();
    } catch (RobotCoreException e) {
      utility.complainToast(e.getMessage(), context);
      DbgLog.error(e.getMessage());
      return;
    } catch (IOException e){
      utility.complainToast("Found " + e.getMessage() + "\n Please fix and re-save", context);
      DbgLog.error(e.getMessage());
      return;
  }
  }

  private void warnIfNoDevices(){
      String msg0 = "No devices found!";
      String msg1 = "In order to use AutoConfigure, please: \n" +
          "   1. Attach a LegacyModuleController, with a \n " +
          "      a. MotorController in port 0, with a \n" +
          "         motor in port 1 and port 2 \n " +
          "      b. ServoController in port 1, with a \n" +
          "         servo in port 1 and port 6 \n"+
          "      c. IR seeker in port 2\n" +
          "      d. Light sensor in port 3 \n" +
          "   2. Press the AutoConfigure button";
    utility.setOrangeText(msg0, msg1, R.id.autoconfigure_info, R.layout.orange_warning, R.id.orangetext0, R.id.orangetext1);
  }

  private void warnWrongDevices(){
    String msg0 = "Wrong devices found!";
    String msg1 = "Found: \n" + scannedDevices.values() + "\n" +
        "Required: \n" +
        "   1. LegacyModuleController, with a \n " +
        "      a. MotorController in port 0, with a \n" +
        "          motor in port 1 and port 2 \n " +
        "      b. ServoController in port 1, with a \n" +
        "          servo in port 1 and port 6 \n"+
        "       c. IR seeker in port 2\n" +
        "       d. Light sensor in port 3 ";
    utility.setOrangeText(msg0, msg1, R.id.autoconfigure_info, R.layout.orange_warning, R.id.orangetext0, R.id.orangetext1);
  }

  private void warnAlreadyConfigured(){
    String msg0 = "Already configured!";
    String msg1 = "To configure again, \n press the AutoConfigure button";
    utility.setOrangeText(msg0, msg1, R.id.autoconfigure_info, R.layout.orange_warning, R.id.orangetext0, R.id.orangetext1);
  }

  // returns true if the right devices have been found and constructed
  private boolean buildK9(){
    boolean needLegacyController = true;
    for (Map.Entry entry : entries){
      DeviceManager.DeviceType type = (DeviceManager.DeviceType) entry.getValue();
      if (type == DeviceType.MODERN_ROBOTICS_USB_LEGACY_MODULE && needLegacyController){
        nameK9Legacy((SerialNumber) entry.getKey());
        needLegacyController = false;
      }
    }
    if (needLegacyController){
      return false;
    }
    LinearLayout autoconfigure_info = (LinearLayout) findViewById(R.id.autoconfigure_info);
    autoconfigure_info.removeAllViews();
    return true;
  }

  private void nameK9Legacy(SerialNumber serialNumber){

    LegacyModuleControllerConfiguration legacyController = (LegacyModuleControllerConfiguration) deviceControllers.get(serialNumber);
    //port 0
    MotorControllerConfiguration motorController = nameMotors(ControllerConfiguration.NO_SERIAL_NUMBER, "motor_1", "motor_2");
    motorController.setPort(0);
    motorController.setName("Legacy Motor Controller");

    //port 1
    ArrayList<String> servoNames = new ArrayList<String>();
    servoNames.add("servo_1");
    servoNames.add(ControllerConfiguration.DISABLED_DEVICE_NAME);
    servoNames.add(ControllerConfiguration.DISABLED_DEVICE_NAME);
    servoNames.add(ControllerConfiguration.DISABLED_DEVICE_NAME);
    servoNames.add(ControllerConfiguration.DISABLED_DEVICE_NAME);
    servoNames.add("servo_6");
    ServoControllerConfiguration servoController = nameServos(ControllerConfiguration.NO_SERIAL_NUMBER, servoNames);
    servoController.setPort(1);
    servoController.setName("Legacy Servo Controller");

    //port 2
    DeviceConfiguration ir_seeker = new DeviceConfiguration(DeviceConfiguration.ConfigurationType.IR_SEEKER);
    ir_seeker.setName("ir_seeker");
    ir_seeker.setPort(2);

    //port 3
    DeviceConfiguration light_sensor = new DeviceConfiguration(DeviceConfiguration.ConfigurationType.LIGHT_SENSOR);
    light_sensor.setName("light_sensor");
    ir_seeker.setPort(3);

    ArrayList<DeviceConfiguration> devices = new ArrayList<DeviceConfiguration>();
    devices.add(motorController);
    devices.add(servoController);
    devices.add(ir_seeker);
    devices.add(light_sensor);
    for (int i = 4; i < 6; i++){
      DeviceConfiguration nothing = new DeviceConfiguration(i);
      devices.add(nothing);
    }
    legacyController.addDevices(devices);
  }

  private MotorControllerConfiguration nameMotors(SerialNumber serialNumber, String port1Name, String port2Name){
    MotorControllerConfiguration motorController;
    if (!serialNumber.equals(ControllerConfiguration.NO_SERIAL_NUMBER)) {
      motorController = (MotorControllerConfiguration) deviceControllers.get(serialNumber);
    } else {
      motorController = new MotorControllerConfiguration();
    }
    ArrayList<DeviceConfiguration> motors = new ArrayList<DeviceConfiguration>();
    MotorConfiguration port1 = new MotorConfiguration(port1Name);
    port1.setPort(1);
    MotorConfiguration port2 = new MotorConfiguration(port2Name);
    port2.setPort(2);
    motors.add(port1);
    motors.add(port2);
    motorController.addMotors(motors);

    return motorController;

  }

  private ServoControllerConfiguration nameServos(SerialNumber serialNumber, ArrayList<String> names){
    ServoControllerConfiguration servoController;
    if (!serialNumber.equals(ControllerConfiguration.NO_SERIAL_NUMBER)) {
      servoController = (ServoControllerConfiguration) deviceControllers.get(serialNumber);
    } else {
      servoController = new ServoControllerConfiguration();
    }
    ArrayList<DeviceConfiguration> servos = new ArrayList<DeviceConfiguration>();
    for (int i = 1; i <= 6; i++){
      ServoConfiguration servo = new ServoConfiguration(names.get(i-1));
      servo.setPort(i);
      servos.add(servo);
    }
    servoController.addServos(servos);
    return servoController;
  }





}
