/* Copyright (c) 2014 Qualcomm Technologies Inc

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
import android.content.Intent;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.view.View;
import android.widget.CheckBox;
import android.widget.EditText;

import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.Serializable;
import java.util.ArrayList;

public class EditServoControllerActivity extends Activity{


  public static final String EDIT_SERVO_ACTIVITY = "Edit Servo ControllerConfiguration Activity";

  private Utility utility;

  private ServoControllerConfiguration servoControllerConfigurationConfig;
  private EditText controller_name;

  private ServoConfiguration servo1 = new ServoConfiguration(1);
  private boolean servo1Enabled = true;
  private CheckBox checkbox_port1;
  private EditText name_servo1;

  private ServoConfiguration servo2 = new ServoConfiguration(2);
  private boolean servo2Enabled = true;
  private CheckBox checkbox_port2;
  private EditText name_servo2;

  private ServoConfiguration servo3 = new ServoConfiguration(3);
  private boolean servo3Enabled = true;
  private CheckBox checkbox_port3;
  private EditText name_servo3;

  private ServoConfiguration servo4 = new ServoConfiguration(4);
  private boolean servo4Enabled = true;
  private CheckBox checkbox_port4;
  private EditText name_servo4;

  private ServoConfiguration servo5 = new ServoConfiguration(5);
  private boolean servo5Enabled = true;
  private CheckBox checkbox_port5;
  private EditText name_servo5;

  private ServoConfiguration servo6 = new ServoConfiguration(6);
  private boolean servo6Enabled = true;
  private CheckBox checkbox_port6;
  private EditText name_servo6;

  @Override
  protected void onCreate(Bundle savedInstanceState){
    super.onCreate(savedInstanceState);
    setContentView(R.layout.servos);

    PreferenceManager.setDefaultValues(this, R.xml.preferences, false);
    utility = new Utility(this);
    RobotLog.writeLogcatToDisk(this, 1024);

    controller_name = (EditText) findViewById(R.id.servocontroller_name);

    name_servo1 = (EditText) findViewById(R.id.editTextResult_servo1);
    checkbox_port1 = (CheckBox) findViewById(R.id.checkbox_port1);

    name_servo2 = (EditText) findViewById(R.id.editTextResult_servo2);
    checkbox_port2 = (CheckBox) findViewById(R.id.checkbox_port2);

    name_servo3 = (EditText) findViewById(R.id.editTextResult_servo3);
    checkbox_port3 = (CheckBox) findViewById(R.id.checkbox_port3);

    name_servo4 = (EditText) findViewById(R.id.editTextResult_servo4);
    checkbox_port4 = (CheckBox) findViewById(R.id.checkbox_port4);

    name_servo5 = (EditText) findViewById(R.id.editTextResult_servo5);
    checkbox_port5 = (CheckBox) findViewById(R.id.checkbox_port5);

    name_servo6 = (EditText) findViewById(R.id.editTextResult_servo6);
    checkbox_port6 = (CheckBox) findViewById(R.id.checkbox_port6);

  }

  @Override
  protected void onStart(){
    super.onStart();

    utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
    Intent intent = getIntent();
    Serializable extra = intent.getSerializableExtra(EDIT_SERVO_ACTIVITY);
    if(extra != null){
      servoControllerConfigurationConfig = (ServoControllerConfiguration)extra;
      ArrayList<DeviceConfiguration> servos = (ArrayList<DeviceConfiguration>) servoControllerConfigurationConfig.getServos();
      servo1 = (ServoConfiguration) servos.get(0);
      servo2 = (ServoConfiguration) servos.get(1);
      servo3 = (ServoConfiguration) servos.get(2);
      servo4 = (ServoConfiguration) servos.get(3);
      servo5 = (ServoConfiguration) servos.get(4);
      servo6 = (ServoConfiguration) servos.get(5);
    }

    controller_name.setText(servoControllerConfigurationConfig.getName());

    //***************** SERVO 1 ********************//
    name_servo1.setText(servo1.getName());
    addListenerOnPort1();
    handleDisabledServo(servo1, checkbox_port1);

  //***************** SERVO 2 ********************//
    name_servo2.setText(servo2.getName());
    addListenerOnPort2();
    handleDisabledServo(servo2, checkbox_port2);

  //***************** SERVO 3 ********************//
    name_servo3.setText(servo3.getName());
    addListenerOnPort3();
    handleDisabledServo(servo3, checkbox_port3);

    //***************** SERVO 4 ********************//
    name_servo4.setText(servo4.getName());
    addListenerOnPort4();
    handleDisabledServo(servo4, checkbox_port4);

    //***************** SERVO 5 ********************//
    name_servo5.setText(servo5.getName());
    addListenerOnPort5();
    handleDisabledServo(servo5, checkbox_port5);

    //***************** SERVO 6 ********************//
    name_servo6.setText(servo6.getName());
    addListenerOnPort6();
    handleDisabledServo(servo6, checkbox_port6);
  }

  private void handleDisabledServo(ServoConfiguration servo, CheckBox checkbox){
    if (servo.getName().equals(DeviceConfiguration.DISABLED_DEVICE_NAME) ||
            servo.getType() == DeviceConfiguration.ConfigurationType.NOTHING){
      checkbox.setChecked(true); // kind of a hack. Sets the checkbox to true, so
                                // when performing the click programmatically,
                                // the checkbox becomes "unclicked" which does the right thing.
      checkbox.performClick();
    } else {
      checkbox.setChecked(true);
    }
  }

  private void addListenerOnPort1(){
    checkbox_port1.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View view) {
        if (((CheckBox)view).isChecked()){
          servo1Enabled = true;
          name_servo1.setEnabled(true);
          name_servo1.setText("");
          servo1.setPort(1);
        } else {
          servo1Enabled = false;
          name_servo1.setEnabled(false);
          name_servo1.setText(DeviceConfiguration.DISABLED_DEVICE_NAME);
        }
      }
    });
  }

  private void addListenerOnPort2(){
    checkbox_port2.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View view) {
        if (((CheckBox)view).isChecked()){
          servo2Enabled = true;
          name_servo2.setEnabled(true);
          name_servo2.setText("");
          servo2.setPort(2);
        } else {
          servo2Enabled = false;
          name_servo2.setEnabled(false);
          name_servo2.setText(DeviceConfiguration.DISABLED_DEVICE_NAME);
        }
      }
    });
  }

  private void addListenerOnPort3(){
    checkbox_port3.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View view) {
        if (((CheckBox)view).isChecked()){
          servo3Enabled = true;
          name_servo3.setEnabled(true);
          name_servo3.setText("");
          servo3.setPort(3);
        } else {
          servo3Enabled = false;
          name_servo3.setEnabled(false);
          name_servo3.setText(DeviceConfiguration.DISABLED_DEVICE_NAME);
        }
      }
    });
  }

  private void addListenerOnPort4(){
    checkbox_port4.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View view) {
        if (((CheckBox)view).isChecked()){
          servo4Enabled = true;
          name_servo4.setEnabled(true);
          name_servo4.setText("");
          servo4.setPort(4);
        } else {
          servo4Enabled = false;
          name_servo4.setEnabled(false);
          name_servo4.setText(DeviceConfiguration.DISABLED_DEVICE_NAME);
        }
      }
    });
  }

  private void addListenerOnPort5(){
    checkbox_port5.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View view) {
        if (((CheckBox)view).isChecked()){
          servo5Enabled = true;
          name_servo5.setEnabled(true);
          name_servo5.setText("");
          servo5.setPort(5);
        } else {
          servo5Enabled = false;
          name_servo5.setEnabled(false);
          name_servo5.setText(DeviceConfiguration.DISABLED_DEVICE_NAME);
        }
      }
    });
  }

  private void addListenerOnPort6(){
    checkbox_port6.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View view) {
        if (((CheckBox)view).isChecked()){
          servo6Enabled = true;
          name_servo6.setEnabled(true);
          name_servo6.setText("");
          servo6.setPort(6);
        } else {
          servo6Enabled = false;
          name_servo6.setEnabled(false);
          name_servo6.setText(DeviceConfiguration.DISABLED_DEVICE_NAME);
        }
      }
    });
  }

  public void saveServoController(View v){
    saveState();
  }

  private void saveState(){
    Intent returnIntent = new Intent();
    ArrayList<DeviceConfiguration> servos = new ArrayList<DeviceConfiguration>();
    if (servo1Enabled){
      String name = name_servo1.getText().toString();
      ServoConfiguration servo1 = new ServoConfiguration(name);
      servo1.setPort(1);
      servos.add(servo1);
    } else { servos.add(new ServoConfiguration(1)); } // add disabled servo

    if (servo2Enabled){
      String name = name_servo2.getText().toString();
      ServoConfiguration servo2 = new ServoConfiguration(name);
      servo2.setPort(2);
      servos.add(servo2);
    } else { servos.add(new ServoConfiguration(2)); } // add disabled servo

    if (servo3Enabled){
      String name = name_servo3.getText().toString();
      ServoConfiguration servo3 = new ServoConfiguration(name);
      servo3.setPort(3);
      servos.add(servo3);
    } else { servos.add(new ServoConfiguration(3)); } // add disabled servo

    if (servo4Enabled){
      String name = name_servo4.getText().toString();
      ServoConfiguration servo4 = new ServoConfiguration(name);
      servo4.setPort(4);
      servos.add(servo4);
    } else { servos.add(new ServoConfiguration(4)); } // add disabled servo

    if (servo5Enabled){
      String name = name_servo5.getText().toString();
      ServoConfiguration servo5 = new ServoConfiguration(name);
      servo5.setPort(5);
      servos.add(servo5);
    } else { servos.add(new ServoConfiguration(5)); } // add disabled servo

    if (servo6Enabled){
      String name = name_servo6.getText().toString();
      ServoConfiguration servo6 = new ServoConfiguration(name);
      servo6.setPort(6);
      servos.add(servo6);
    } else { servos.add(new ServoConfiguration(6)); } // add disabled servo

    servoControllerConfigurationConfig.addServos(servos);
    servoControllerConfigurationConfig.setName(controller_name.getText().toString());
    returnIntent.putExtra(EDIT_SERVO_ACTIVITY, servoControllerConfigurationConfig);
    setResult(RESULT_OK, returnIntent);
    finish();
  }

  public void cancelServoController(View v){
    setResult(RESULT_CANCELED, new Intent());
    finish();
  }

}