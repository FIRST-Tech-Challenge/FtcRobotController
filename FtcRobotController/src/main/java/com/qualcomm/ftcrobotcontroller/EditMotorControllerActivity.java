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
import com.qualcomm.robotcore.hardware.configuration.MotorConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.Serializable;
import java.util.ArrayList;

public class EditMotorControllerActivity extends Activity{

  private Utility utility;
  public static final String EDIT_MOTOR_CONTROLLER_CONFIG = "EDIT_MOTOR_CONTROLLER_CONFIG";

  private MotorControllerConfiguration motorControllerConfigurationConfig;
  private ArrayList<DeviceConfiguration> motors = new ArrayList<DeviceConfiguration>();
  private MotorConfiguration motor1 = new MotorConfiguration(1);
  private MotorConfiguration motor2 = new MotorConfiguration(2);

  private EditText controller_name;

  private boolean motor1Enabled = true;
  private boolean motor2Enabled = true;

  private CheckBox checkbox_motor1;
  private CheckBox checkbox_motor2;

  private EditText motor1_name;
  private EditText motor2_name;

  @Override
  protected void onCreate(Bundle savedInstanceState){
    super.onCreate(savedInstanceState);
    setContentView(R.layout.motors);

    PreferenceManager.setDefaultValues(this, R.xml.preferences, false);
    utility = new Utility(this);

    RobotLog.writeLogcatToDisk(this, 1024);

    controller_name = (EditText) findViewById(R.id.controller_name);
    checkbox_motor1 = (CheckBox) findViewById(R.id.checkbox_port1);
    checkbox_motor2 = (CheckBox) findViewById(R.id.checkbox_port2);
    motor1_name = (EditText) findViewById(R.id.editTextResult_motor1);
    motor2_name = (EditText) findViewById(R.id.editTextResult_motor2);

  }

  @Override
  protected void onStart(){
    super.onStart();

    utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
    Intent intent = getIntent();
    Serializable extra = intent.getSerializableExtra(EDIT_MOTOR_CONTROLLER_CONFIG);

    if(extra != null) {
      motorControllerConfigurationConfig = (MotorControllerConfiguration) extra;
      motors = (ArrayList<DeviceConfiguration>) motorControllerConfigurationConfig.getMotors();
      motor1 = (MotorConfiguration) motors.get(0);
      motor2 = (MotorConfiguration) motors.get(1);

      controller_name.setText(motorControllerConfigurationConfig.getName());

      motor1_name.setText(motor1.getName());
      motor2_name.setText(motor2.getName());

      addListenerOnPort1();
      handleDisabledMotor(motor1, checkbox_motor1);
      addListenerOnPort2();
      handleDisabledMotor(motor2, checkbox_motor2);
    }
  }
  private void handleDisabledMotor(MotorConfiguration motor, CheckBox checkbox){
    if (motor.getName().equals(DeviceConfiguration.DISABLED_DEVICE_NAME) ||
            motor.getType() == DeviceConfiguration.ConfigurationType.NOTHING){
      checkbox.setChecked(true); // kind of a hack. Sets the checkbox to true, so
                                // when performing the click programmatically,
                               // the checkbox becomes "unclicked" which does the right thing.
      checkbox.performClick();
    } else {
      checkbox.setChecked(true);
    }
  }

  private void addListenerOnPort1(){
    checkbox_motor1.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View view) {
        if (((CheckBox)view).isChecked()){
          motor1Enabled = true;
          motor1_name.setEnabled(true);
          motor1_name.setText("");
          motor1.setPort(1);
          motor1.setType(DeviceConfiguration.ConfigurationType.MOTOR);
        } else {
          motor1Enabled = false;
          motor1_name.setEnabled(false);
          motor1_name.setText(DeviceConfiguration.DISABLED_DEVICE_NAME);
          motor1.setType(DeviceConfiguration.ConfigurationType.NOTHING);
        }
      }
    });
  }

  private void addListenerOnPort2(){
    checkbox_motor2.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View view) {
        if (((CheckBox)view).isChecked()){
          motor2Enabled = true;
          motor2_name.setEnabled(true);
          motor2_name.setText("");
          motor2.setPort(2);
          motor1.setType(DeviceConfiguration.ConfigurationType.MOTOR);
        } else {
          motor2Enabled = false;
          motor2_name.setEnabled(false);
          motor2_name.setText(DeviceConfiguration.DISABLED_DEVICE_NAME);
          motor1.setType(DeviceConfiguration.ConfigurationType.NOTHING);
        }
      }
    });
  }

  public void saveMotorController(View v){
    saveState();
  }

  private void saveState(){

    Intent returnIntent = new Intent();
    ArrayList<DeviceConfiguration> motors = new ArrayList<DeviceConfiguration>();
    if (motor1Enabled){
      String name = motor1_name.getText().toString();
      MotorConfiguration newMotor = new MotorConfiguration(name);
      newMotor.setDisabled(false);
      newMotor.setPort(1);
      motors.add(newMotor);
    } else { motors.add(new MotorConfiguration(1)); } // add disabled motor

    if (motor2Enabled){
      String name = motor2_name.getText().toString();
      MotorConfiguration newMotor = new MotorConfiguration(name);
      newMotor.setDisabled(false);
      newMotor.setPort(2);
      motors.add(newMotor);
    } else { motors.add(new MotorConfiguration(2)); } // add disabled motor

    motorControllerConfigurationConfig.addMotors(motors);
    motorControllerConfigurationConfig.setName(controller_name.getText().toString());
    returnIntent.putExtra(EDIT_MOTOR_CONTROLLER_CONFIG, motorControllerConfigurationConfig);

    setResult(RESULT_OK, returnIntent);
    finish();

  }

  public void cancelMotorController(View v){
    setResult(RESULT_CANCELED, new Intent());
    finish();
  }


}