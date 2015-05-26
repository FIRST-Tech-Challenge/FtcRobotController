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
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.Spinner;
import android.widget.TextView;

import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.LegacyModuleControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;

import java.io.Serializable;
import java.util.ArrayList;

public class EditLegacyModuleControllerActivity extends Activity{

  private Utility utility;
  private String activeFilename;
  private Context context;

  public static final String EDIT_LEGACY_CONFIG = "EDIT_LEGACY_CONFIG";
  public static final int EDIT_MOTOR_CONTROLLER_REQUEST_CODE = 101;
  public static final int EDIT_SERVO_CONTROLLER_REQUEST_CODE = 102;
  private LegacyModuleControllerConfiguration legacyModuleControllerConfiguration;
  private EditText controller_name;
  private ArrayList<DeviceConfiguration> modules = new ArrayList<DeviceConfiguration>();

  private DeviceConfiguration module0;
  private View info_port0;

  private DeviceConfiguration module1;
  private View info_port1;

  private DeviceConfiguration module2;
  private View info_port2;

  private DeviceConfiguration module3;
  private View info_port3;

  private DeviceConfiguration module4;
  private View info_port4;

  private DeviceConfiguration module5;
  private View info_port5;


  /**
   * In onCreate, we gather all of the linearLayout's that are associated with each port.
   * this is how the legacy_module.xml file is reused, but we read and write to the correct
   * Spinners, EditTexts, TextViews, and Buttons. The TextView port# is set during onCreate
   * as a way to "name" that chunk of xml code. Each layout is then identified by the port number.
   */
  @Override
  protected void onCreate(Bundle savedInstanceState){
    super.onCreate(savedInstanceState);
    setContentView(R.layout.legacy);

    LinearLayout layout_port0 = (LinearLayout) findViewById(R.id.linearLayout0);
    info_port0 = getLayoutInflater().inflate(R.layout.legacy_module, layout_port0, true);
    TextView port0 = (TextView) info_port0.findViewById(R.id.portNumber);
    port0.setText("0");

    LinearLayout layout_port1 = (LinearLayout) findViewById(R.id.linearLayout1);
    info_port1 = getLayoutInflater().inflate(R.layout.legacy_module, layout_port1, true);
    TextView port1 = (TextView) info_port1.findViewById(R.id.portNumber);
    port1.setText("1");

    LinearLayout layout_port2 = (LinearLayout) findViewById(R.id.linearLayout2);
    info_port2 = getLayoutInflater().inflate(R.layout.legacy_module, layout_port2, true);
    TextView port2 = (TextView) info_port2.findViewById(R.id.portNumber);
    port2.setText("2");

    LinearLayout layout_port3 = (LinearLayout) findViewById(R.id.linearLayout3);
    info_port3 = getLayoutInflater().inflate(R.layout.legacy_module, layout_port3, true);
    TextView port3 = (TextView) info_port3.findViewById(R.id.portNumber);
    port3.setText("3");

    LinearLayout layout_port4 = (LinearLayout) findViewById(R.id.linearLayout4);
    info_port4 = getLayoutInflater().inflate(R.layout.legacy_module, layout_port4, true);
    TextView port4 = (TextView) info_port4.findViewById(R.id.portNumber);
    port4.setText("4");

    LinearLayout layout_port5 = (LinearLayout) findViewById(R.id.linearLayout5);
    info_port5 = getLayoutInflater().inflate(R.layout.legacy_module, layout_port5, true);
    TextView port5 = (TextView) info_port5.findViewById(R.id.portNumber);
    port5.setText("5");

    this.context = this;
    PreferenceManager.setDefaultValues(this, R.xml.preferences, false);
    utility = new Utility(this);
    RobotLog.writeLogcatToDisk(this, 1024);
    controller_name = (EditText) findViewById(R.id.legacycontroller_name);


  }

  @Override
  protected void onStart(){
    super.onStart();

    utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
    activeFilename = utility.getFilenameFromPrefs(R.string.pref_hardware_config_filename, Utility.NO_FILE);

    Intent intent = getIntent();
    Serializable extra = intent.getSerializableExtra(EDIT_LEGACY_CONFIG);
    if(extra != null) {
      legacyModuleControllerConfiguration = (LegacyModuleControllerConfiguration) extra;
      modules = (ArrayList<DeviceConfiguration>) legacyModuleControllerConfiguration.getDevices();
      module0 = modules.get(0);
      module1 = modules.get(1);
      module2 = modules.get(2);
      module3 = modules.get(3);
      module4 = modules.get(4);
      module5 = modules.get(5);

      controller_name.setText(legacyModuleControllerConfiguration.getName());

      // for loop
      populatePort(info_port0, module0);
      populatePort(info_port1, module1);
      populatePort(info_port2, module2);
      populatePort(info_port3, module3);
      populatePort(info_port4, module4);
      populatePort(info_port5, module5);

    }
  }

  @Override
  protected void onActivityResult(int requestCode, int resultCode, Intent data) {

    Serializable extra = null;
    if (resultCode == RESULT_OK){
      if (requestCode == EDIT_MOTOR_CONTROLLER_REQUEST_CODE){
        extra = data.getSerializableExtra(EditMotorControllerActivity.EDIT_MOTOR_CONTROLLER_CONFIG);
      } else if (requestCode == EDIT_SERVO_CONTROLLER_REQUEST_CODE){
        extra = data.getSerializableExtra(EditServoControllerActivity.EDIT_SERVO_ACTIVITY);
      }

      if (extra != null){
        ControllerConfiguration newC = (ControllerConfiguration) extra;
        setModule(newC);
        populatePort(findViewByPort(newC.getPort()), findModuleByPort(newC.getPort()));
        String name = activeFilename;
        // only update the filename if it hasn't already been updated to have "unsaved" in it
        if (!name.toLowerCase().contains(Utility.UNSAVED.toLowerCase())){
          name = Utility.UNSAVED + " " + activeFilename;
          utility.saveToPreferences(name, R.string.pref_hardware_config_filename);
          activeFilename = name;
        }
        utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
      }
    }
  }

  public void saveLegacyController(View v){
    saveState();
  }

  private void saveState(){
    Intent returnIntent = new Intent();
    ArrayList<DeviceConfiguration> devices = new ArrayList<DeviceConfiguration>();

    for (int i = 0; i <6; i++){
      View currentLayout = findViewByPort(i);
      EditText name = (EditText) currentLayout.findViewById(R.id.editTextResult_name);
      Spinner choiceSpinner = (Spinner) currentLayout.findViewById(R.id.choiceSpinner);
      String type = choiceSpinner.getSelectedItem().toString();

      if (!type.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.NOTHING.toString())){
        DeviceConfiguration device = buildDevice(type, name.getText().toString(), findModuleByPort(i), i);
        devices.add(device);
      } else { devices.add(new DeviceConfiguration(i)); } // add disabled device
    }

    legacyModuleControllerConfiguration.addDevices(devices);
    legacyModuleControllerConfiguration.setName(controller_name.getText().toString());

    returnIntent.putExtra(EDIT_LEGACY_CONFIG, legacyModuleControllerConfiguration);
    setResult(RESULT_OK, returnIntent);
    finish();
  }

  public void cancelLegacyController(View v){
    setResult(RESULT_CANCELED, new Intent());
    finish();
  }

  /****************** HEAVY LIFTING ************************/

  /**
   * This method loads information from the module (Gyro, Accelerometer, Motor Controller, etc) into
   * the xml file that's associated with that port.
   *
   * @param v - the LinearLayout that loaded the legacy_module.xml code
   * @param module - the global module whose information should populate that port
   */
  private void populatePort(View v, DeviceConfiguration module){

    Spinner choiceSpinner = (Spinner) v.findViewById(R.id.choiceSpinner);
    ArrayAdapter choice_adapter = (ArrayAdapter) choiceSpinner.getAdapter();

    int spinnerPosition = choice_adapter.getPosition(module.getType().toString());
    choiceSpinner.setSelection(spinnerPosition);
    choiceSpinner.setOnItemSelectedListener(choiceListener);

    String name = module.getName();
    EditText nameText = (EditText) v.findViewById(R.id.editTextResult_name);

    TextView portNumber = (TextView) v.findViewById(R.id.portNumber);
    int port = Integer.parseInt(portNumber.getText().toString());
    nameText.addTextChangedListener(new UsefulTextWatcher(findViewByPort(port)));
    nameText.setText(name);
  }

  /**
   * Sets device to a non-attached device (name "NO DEVICE ATTACHED" and name field grayed out).
   * Removes the button if necessary.
   * @param layout - the view that holds all the necessary UI elements
   */
  private void clearDevice(LinearLayout layout){
    TextView portNumber = (TextView) layout.findViewById(R.id.portNumber);
    int port = Integer.parseInt(portNumber.getText().toString());
    EditText nameText = (EditText) layout.findViewById(R.id.editTextResult_name);

    nameText.setEnabled(false);
    nameText.setText(DeviceConfiguration.DISABLED_DEVICE_NAME);

    DeviceConfiguration newModule = new DeviceConfiguration(DeviceConfiguration.ConfigurationType.NOTHING);
    newModule.setPort(port);
    setModule(newModule);

    setButtonVisibility(port, View.INVISIBLE);
  }

  /**
   * Updates the module behind the scenes to the type that was selected on the spinner.
   * @param layout - - the view that holds all the necessary UI elements
   * @param newType - the new type that was just selected
   */
  private void changeDevice(LinearLayout layout, String newType){
    TextView portNumber = (TextView) layout.findViewById(R.id.portNumber);
    int port = Integer.parseInt(portNumber.getText().toString());
    EditText nameText = (EditText) layout.findViewById(R.id.editTextResult_name);
    DeviceConfiguration currentModule = findModuleByPort(port);

    nameText.setEnabled(true);
    clearNameIfNecessary(nameText, currentModule);

    DeviceConfiguration.ConfigurationType type = currentModule.typeFromString(newType);
    if (type == DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER ||
            type == DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER){
      createController(port, newType);
      setButtonVisibility(port, View.VISIBLE);
    } else {
      currentModule.setType(type);
      setButtonVisibility(port, View.INVISIBLE);
    }

  }


  /**
   * If the drop-down spinner-selected item is a Controller (Motor- or Servo-), we need to create
   * an empty Controller with the proper number of motors/servos.
   * @param port - the port where this controller got added
   * @param newType - the type of controller we're creating
   */
  private void createController(int port, String newType){

    DeviceConfiguration currentModule = findModuleByPort(port);

    String name = currentModule.getName();
    ArrayList<DeviceConfiguration> devices = new ArrayList<DeviceConfiguration>();
    SerialNumber serialNumber = ControllerConfiguration.NO_SERIAL_NUMBER;

    DeviceConfiguration.ConfigurationType currentType = currentModule.getType();
    if (!currentType.toString().equalsIgnoreCase(newType)) { //only update the controller if it's a new choice.
      ControllerConfiguration newModule = new ControllerConfiguration("dummy module",
                                                 devices, serialNumber, DeviceConfiguration.ConfigurationType.NOTHING);
      if (newType.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER.toString())) {
        for (int i = 1; i <= 2; i++) {
          devices.add(new MotorConfiguration(i));
        }
        newModule = new MotorControllerConfiguration(name, devices, serialNumber);
        newModule.setPort(port);
      }
      if (newType.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER.toString())) {
        for (int i = 1; i <= 6; i++) {
          devices.add(new ServoConfiguration(i));
        }
        newModule = new ServoControllerConfiguration(name, devices, serialNumber);
        newModule.setPort(port);
      }
      setModule(newModule);
    }
  }

  /**
   * When the drop-down item is a Controller, this button appears. Clicking it launches the
   * appropriate activity
   * @param v - the button that got pressed
   */
  public void editController_portALL(View v){
    //view is Button
    //view.getParent is RelativeLayout
    //view.getparent.getparent is the LinearLayout around the whole module zone
    LinearLayout layout = (LinearLayout) v.getParent().getParent();
    TextView portNumber = (TextView) layout.findViewById(R.id.portNumber);
    int port = Integer.parseInt(portNumber.getText().toString());
    DeviceConfiguration currentModule = findModuleByPort(port);

    if (!isController(currentModule)){
      Spinner choiceSpinner = (Spinner) layout.findViewById(R.id.choiceSpinner);
      String type = choiceSpinner.getSelectedItem().toString();
      createController(port, type);
    }
    editController_general(currentModule);
  }

  /**
   * Launches the activity for the controller
   * @param module - the module we're about to edit
   */
  private void editController_general(DeviceConfiguration module){
    //names already gone
    LinearLayout layout = (LinearLayout) findViewByPort(module.getPort());
    EditText nameText = (EditText) layout.findViewById(R.id.editTextResult_name);
    module.setName(nameText.getText().toString());

    if(module.getType() == DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER){
      int requestCode = EDIT_MOTOR_CONTROLLER_REQUEST_CODE;
      Intent editMotorControllerIntent = new Intent(context, EditMotorControllerActivity.class);
      editMotorControllerIntent.putExtra(EditMotorControllerActivity.EDIT_MOTOR_CONTROLLER_CONFIG, module);
      editMotorControllerIntent.putExtra("requestCode", requestCode);
      setResult(RESULT_OK, editMotorControllerIntent);
      startActivityForResult(editMotorControllerIntent, requestCode);
    }
    if (module.getType() == DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER){
      int requestCode = EDIT_SERVO_CONTROLLER_REQUEST_CODE;
      Intent editServoControllerIntent = new Intent(context, EditServoControllerActivity.class);
      editServoControllerIntent.putExtra(EditServoControllerActivity.EDIT_SERVO_ACTIVITY, module);
      setResult(RESULT_OK, editServoControllerIntent);
      startActivityForResult(editServoControllerIntent, requestCode);
    }
  }

  /**
   * Used while saving state. This gets all of the information from the UI elements, and
   * creates the appropriate objects.
   * @param type - type of device
   * @param name - the name of the device
   * @param module - the actual module itself
   * @param port - the port it's attached to
   * @return the device we just created from the UI elements
   */
  protected DeviceConfiguration buildDevice(String type, String name, DeviceConfiguration module, int port) {
    DeviceConfiguration device;
    if (type.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.MOTOR.toString())) {
      MotorConfiguration motor = new MotorConfiguration(name);
      motor.setPort(port);
      return motor;
    }
    if (type.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.SERVO.toString())) {
      ServoConfiguration servo = new ServoConfiguration(name);
      servo.setPort(port);
      return servo;
    }
    if (type.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER.toString())) {
      device = new MotorControllerConfiguration(name,
              ((MotorControllerConfiguration) module).getMotors(),
              ((MotorControllerConfiguration) module).getSerialNumber());
      device.setPort(port);
      return device;
    }
    if (type.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER.toString())) {
      device = new ServoControllerConfiguration(name,
              ((ServoControllerConfiguration) module).getServos(),
              ((ServoControllerConfiguration) module).getSerialNumber());
      device.setPort(port);
      return device;
    } else {
      device = new DeviceConfiguration(module.typeFromString(type));
      device.setName(name);
      device.setPort(port);
      return device;
    }
  }

  /**
   * Update the master list, and the global variables referenced everywhere
   * @param dev - the new device we're updating to.
   */
  private DeviceConfiguration setModule(DeviceConfiguration dev){
    int port = dev.getPort();
    switch (port){
      case 0:
        module0 = dev;
        for (DeviceConfiguration device : modules){
          if (device.getPort() == 0){
            modules.remove(device);
            modules.add(0, module0);
            break;
          }
        }
        return module0;
      case 1:
        module1 = dev;
        for (DeviceConfiguration device : modules){
          if (device.getPort() == 1){
            modules.remove(device);
            modules.add(1, module1);
            break;
          }
        }
        return module1;
      case 2:
        module2 = dev;
        for (DeviceConfiguration device : modules){
          if (device.getPort() == 2){
            modules.remove(device);
            modules.add(2, module2);
            break;
          }
        }
        return module2;
      case 3:
        module3 = dev;
        for (DeviceConfiguration device : modules){
          if (device.getPort() == 3){
            modules.remove(device);
            modules.add(3, module3);
            break;
          }
        }
        return module3;
      case 4:
        module4 = dev;
        for (DeviceConfiguration device : modules){
          if (device.getPort() == 4){
            modules.remove(device);
            modules.add(4, module4);
            break;
          }
        }
        return module4;
      case 5:
        module5 = dev;
        for (DeviceConfiguration device : modules){
          if (device.getPort() == 5){
            modules.remove(device);
            modules.add(5, module5);
            break;
          }
        }
        return module5;

      default:
        return null;
    }
  }

  /****************** SIMPLE UTILITIES ********************/

  private View findViewByPort(int port){
    switch (port){
      case 0:
        return info_port0;
      case 1:
        return info_port1;
      case 2:
        return info_port2;
      case 3:
        return info_port3;
      case 4:
        return info_port4;
      case 5:
        return info_port5;
      default:
        return null;
    }
  }

  public DeviceConfiguration findModuleByPort(int port){
    switch(port){
      case 0:
        return module0;
      case 1:
        return module1;
      case 2:
        return module2;
      case 3:
        return module3;
      case 4:
        return module4;
      case 5:
        return module5;
      default:
        return null;
    }
  }

  /**
   * Makes the "edit controller" button visible or invisible.
   * @param port - the port that now holds a controller
   * @param visibility - whether or not the button should be visible
   */
  private void setButtonVisibility(int port, int visibility ){
    View layout = findViewByPort(port);
    Button button = (Button) layout.findViewById(R.id.edit_controller_btn);
    button.setVisibility(visibility);
  }

  /**
   *
   * @param module - the module in question
   * @return - whether or not it's a controller, based on device type
   */
  private boolean isController(DeviceConfiguration module){
    return (module.getType() == DeviceConfiguration.ConfigurationType.MOTOR_CONTROLLER ||
            module.getType() == DeviceConfiguration.ConfigurationType.SERVO_CONTROLLER);
  }

  /**
   * This gets called if we're changing the device, so if the nametext was "NO DEVICE ATTACHED",
   * it should now be empty for editing. Otherwise, it should be the name of the module.
   * @param nameText - the name field that may need to be "cleared"
   * @param module - the module
   */
  private void clearNameIfNecessary(EditText nameText, DeviceConfiguration module){
    if (nameText.getText().toString().equalsIgnoreCase(DeviceConfiguration.DISABLED_DEVICE_NAME)) {
      nameText.setText("");
      module.setName("");
    } else {
      nameText.setText(module.getName());
    }
  }

  /*********************** HELPER OBJECT **************************/

  /**
   * The listener that controls the behavior when an item is selected.
   */
  private AdapterView.OnItemSelectedListener choiceListener = new AdapterView.OnItemSelectedListener() {
    @Override
    public void onItemSelected(AdapterView<?> parent, View view, int pos, long l) {

      String selected = parent.getItemAtPosition(pos).toString();

      //view is SpinnerItem
      //view.getParent is Spinner
      //view.getparent.getparent is the RelativeLayout around the Spinner!!!!!!!!!!
      //view.getparent.getparent.getparent is the Linearlayout around the whole module zone
      LinearLayout layout = (LinearLayout) view.getParent().getParent().getParent();

      if (selected.equalsIgnoreCase(DeviceConfiguration.ConfigurationType.NOTHING.toString())){
        clearDevice(layout);
      }  else {
        changeDevice(layout, selected);
      }
    }

    @Override
    public void onNothingSelected(AdapterView<?> adapterView) {}
  };

  /***************************** Private inner class ***************************/

  /**
   * For some reason, the default TextWatcher in Android does not give you any way to access
   * the view around the text field in question. This inner class is instantiated with the
   * correct view, which can be used to find the correct module, so we can update the module's
   * name as soon as the user types it in.
   */
  private class UsefulTextWatcher implements TextWatcher {

    private int port;
    private UsefulTextWatcher(View layout) {
      TextView portNumber = (TextView) layout.findViewById(R.id.portNumber);
      port = Integer.parseInt(portNumber.getText().toString());
    }

    public void beforeTextChanged(CharSequence charSequence, int i, int i1, int i2) {}
    public void onTextChanged(CharSequence charSequence, int i, int i1, int i2) {}

    public void afterTextChanged(Editable editable) {
      DeviceConfiguration module = findModuleByPort(port);
      String text = editable.toString();
      module.setName(text);
    }
  }

}