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
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ListView;
import android.widget.TextView;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.modernrobotics.ModernRoboticsDeviceManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.DeviceManager.DeviceType;
import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration.ConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.DeviceInfoAdapter;
import com.qualcomm.robotcore.hardware.configuration.ReadXMLFileHandler;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

public class FtcConfigurationActivity extends Activity{

  private static final int EDIT_MOTOR_CONTROLLER = 1;
  private static final int EDIT_SERVO_CONTROLLER = 2;
  private static final int EDIT_LEGACY_MODULE_CONTROLLER = 3;

  private Thread t;

  private Map<SerialNumber, ControllerConfiguration> deviceControllers = new HashMap<SerialNumber, ControllerConfiguration>();
  private Context context;
  private DeviceManager deviceManager;
  private Button scanButton;
  private String preferredFilename = Utility.NO_FILE;
  protected Map<SerialNumber, DeviceType> scannedDevices = new HashMap<SerialNumber, DeviceType>();
  protected Set<Entry<SerialNumber, DeviceType>> entries = new HashSet<Entry<SerialNumber, DeviceType>>();
  protected SharedPreferences preferences;
  private Utility utility;

  @Override
  protected void onCreate(Bundle savedInstanceState){
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_ftc_configuration);

    RobotLog.writeLogcatToDisk(this, 1024);
    this.context = this;
    utility = new Utility(this);
    scanButton = (Button) findViewById(R.id.scanButton);
    buildInfoButtons();

    try{
      deviceManager = new ModernRoboticsDeviceManager(context, null);
      //deviceManager = new MockDeviceManager(context, null);
    }catch (RobotCoreException e){
      utility.complainToast("Failed to open the Device Manager", context);
      DbgLog.error("Failed to open deviceManager: " + e.toString());
      DbgLog.logStacktrace(e);
    }

    preferences = PreferenceManager.getDefaultSharedPreferences(this);
  }

  DialogInterface.OnClickListener close_ok_listener = new DialogInterface.OnClickListener(){
    public void onClick(DialogInterface dialog, int button){
      //do nothing
    }
  };

  private void buildInfoButtons(){
    Button devicesInfoButton = (Button) findViewById(R.id.devices_holder).findViewById(R.id.info_btn);
    devicesInfoButton.setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(View view) {
        AlertDialog.Builder builder = utility.buildBuilder("Devices", "These are the devices " +
            "discovered by the Hardware Wizard. You can click on the name of each device to edit" +
            " the information relating to that device. Make sure each device has a unique name. " +
            "The names should match what is in the Op mode code. Scroll down to see more " +
            "devices if there are any.");
        builder.setPositiveButton("Ok", close_ok_listener);
        AlertDialog alert = builder.create();
        alert.show();
        TextView textView = (TextView) alert.findViewById(android.R.id.message);
        textView.setTextSize(14);
      }
    });

    Button saveConfigButton = (Button) findViewById(R.id.save_holder).findViewById(R.id.info_btn);
    saveConfigButton.setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(View view) {
        AlertDialog.Builder builder = utility.buildBuilder("Save Configuration", "Clicking the " +
            "save button will create an xml file in: \n      /sdcard/FIRST/  \nThis file will be used to " +
            "initialize the robot.");
        builder.setPositiveButton("Ok", close_ok_listener);
        AlertDialog alert = builder.create();
        alert.show();
        TextView textView = (TextView) alert.findViewById(android.R.id.message);
        textView.setTextSize(14);
      }
    });
  }

  @Override
  protected void onStart() {
    super.onStart();

    preferredFilename = utility.getFilenameFromPrefs(R.string.pref_hardware_config_filename, Utility.NO_FILE);
    if (preferredFilename.equalsIgnoreCase(Utility.NO_FILE)){
      utility.createConfigFolder();
    }

    utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
    warnIfNoDevices();

    //if the preferred filename has been changed, we shouldn't read from the original file
    // that would overwrite all the changes.
    if (!preferredFilename.toLowerCase().contains(Utility.UNSAVED.toLowerCase())){
      readFile();
    }

    scanButton.setOnClickListener(new OnClickListener(){

      @Override
      public void onClick(View view) {

      t = new Thread(new Runnable() {
        @Override
        public void run() {
        try{
          DbgLog.msg("Scanning USB bus");
          scannedDevices = deviceManager.scanForUsbDevices();

        }catch (RobotCoreException e){
          DbgLog.error("Device scan failed");
        }

        runOnUiThread(new Runnable() {
          @Override
          public void run() {
          utility.resetCount();
          clearDuplicateWarning();
          utility.saveToPreferences(Utility.NO_FILE, R.string.pref_hardware_config_filename);
          preferredFilename = Utility.NO_FILE;
          utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
          entries = scannedDevices.entrySet();

          //if you just scanned, wipe out the previous devices
          deviceControllers = new HashMap<SerialNumber, ControllerConfiguration>();
          utility.createLists(entries, deviceControllers);
          populateList();
          warnIfNoDevices();
          }
        });
        }
      });
        alertBeforeScan();
      }
    });
  }

  private void alertBeforeScan(){
    if (preferredFilename.toLowerCase().contains(Utility.UNSAVED.toLowerCase())){

      String message = "If you scan, your current unsaved changes will be lost.";
      final EditText input = new EditText(context);
      input.setEnabled(false);
      input.setText("");
      AlertDialog.Builder builder = utility.buildBuilder("Unsaved changes", message);
      builder.setView(input);
      DialogInterface.OnClickListener ok_listener = new DialogInterface.OnClickListener(){
        public void onClick(DialogInterface dialog, int button){
          t.start();
        }
      };

      builder.setPositiveButton("Ok", ok_listener);
      builder.setNegativeButton("Cancel", do_nothing_cancel_listener);
      builder.show();
    } else {
      t.start();
    }
  }

  /**
   * This method parses the XML of the active configuration file, and calls methods to populate
   * the appropriate data structures to the configuration information can be displayed to the
   * user.
   */
  private void readFile(){
    ReadXMLFileHandler parser = new ReadXMLFileHandler(context);

    if (preferredFilename.equalsIgnoreCase(Utility.NO_FILE)) {
      // don't try to parse if there's no file
      return;
    }

    InputStream inputStream = null;
    try {
      inputStream = new FileInputStream(Utility.CONFIG_FILES_DIR + preferredFilename + Utility.FILE_EXT);
    } catch (FileNotFoundException e) {
      DbgLog.error("File was not found: " + preferredFilename);
      DbgLog.logStacktrace(e);
      utility.complainToast("That file was not found: " + preferredFilename, context);
      return;
    }
    ArrayList<ControllerConfiguration> controllerList = (ArrayList<ControllerConfiguration>) parser.parse(inputStream);
    buildHashMap(controllerList);
    populateList();
    warnIfNoDevices();
  }


  private void warnIfNoDevices(){
    if (deviceControllers.size() == 0){
      String msg0 ="No devices found!";
      String msg1 = "In order to find devices: \n" +
          "    1. Attach a robot \n " +
          "   2. Press the 'Scan' button";
      utility.setOrangeText(msg0, msg1, R.id.empty_devicelist, R.layout.orange_warning, R.id.orangetext0, R.id.orangetext1);
      clearDuplicateWarning();
    } else {
      LinearLayout empty_devicelist = (LinearLayout) findViewById(R.id.empty_devicelist);
      empty_devicelist.removeAllViews();
      empty_devicelist.setVisibility(View.GONE);
    }
  }

  private void warnDuplicateNames(String dupeMsg){
    String msg0 ="Found " + dupeMsg;
    String msg1 = "Please fix and re-save.";
    utility.setOrangeText(msg0, msg1, R.id.warning_layout, R.layout.orange_warning, R.id.orangetext0, R.id.orangetext1);
  }

  private void clearDuplicateWarning(){
    LinearLayout warning_layout = (LinearLayout) findViewById(R.id.warning_layout);
    warning_layout.removeAllViews();
    warning_layout.setVisibility(View.GONE);
  }


  /**
   * Populates the list with the relevant controllers from the deviceControllers variable.
   * That variable is either from scanned devices, or read in from an xml file.
   */
  private void populateList() {

    ListView controllerListView = (ListView) findViewById(R.id.controllersList);
    DeviceInfoAdapter adapter = new DeviceInfoAdapter(this, android.R.layout.simple_list_item_1, android.R.id.text1, deviceControllers);
    controllerListView.setAdapter(adapter);

    controllerListView.setOnItemClickListener(new OnItemClickListener() {

      @Override
      public void onItemClick(AdapterView<?> adapterView, View v, int pos, long arg3) {
        ControllerConfiguration item = (ControllerConfiguration) adapterView.getItemAtPosition(pos);
        ConfigurationType itemType = item.getType();
        if (itemType.equals(ConfigurationType.MOTOR_CONTROLLER)){
          int requestCode = EDIT_MOTOR_CONTROLLER;
          Intent editMotorControllerIntent = new Intent(context, EditMotorControllerActivity.class);
          editMotorControllerIntent.putExtra(EditMotorControllerActivity.EDIT_MOTOR_CONTROLLER_CONFIG, item);
          editMotorControllerIntent.putExtra("requestCode", requestCode);
          setResult(RESULT_OK, editMotorControllerIntent);
          startActivityForResult(editMotorControllerIntent, requestCode);
        }
        if (itemType.equals(ConfigurationType.SERVO_CONTROLLER)){
          int requestCode = EDIT_SERVO_CONTROLLER;
          Intent editServoControllerIntent = new Intent(context, EditServoControllerActivity.class);
          editServoControllerIntent.putExtra(EditServoControllerActivity.EDIT_SERVO_ACTIVITY, item);
          editServoControllerIntent.putExtra("requestCode", requestCode);
          setResult(RESULT_OK, editServoControllerIntent);
          startActivityForResult(editServoControllerIntent, requestCode);
        }
        if (itemType.equals(ConfigurationType.LEGACY_MODULE_CONTROLLER)){
          int requestCode = EDIT_LEGACY_MODULE_CONTROLLER;
          Intent editLegacyControllerIntent = new Intent(context, EditLegacyModuleControllerActivity.class);
          editLegacyControllerIntent.putExtra(EditLegacyModuleControllerActivity.EDIT_LEGACY_CONFIG, item);
          editLegacyControllerIntent.putExtra("requestCode", requestCode);
          setResult(RESULT_OK, editLegacyControllerIntent);
          startActivityForResult(editLegacyControllerIntent, requestCode);
        }
      }

    });

  }

  @Override
  protected void onActivityResult(int requestCode, int resultCode, Intent data) {

    if (resultCode == RESULT_CANCELED){
      return;
    }
    Serializable extra = null;
    if (requestCode == 1) {
      extra = data.getSerializableExtra(EditMotorControllerActivity.EDIT_MOTOR_CONTROLLER_CONFIG);
    } else if (requestCode == 2) {
      extra = data.getSerializableExtra(EditServoControllerActivity.EDIT_SERVO_ACTIVITY);
    } else if (requestCode == 3) {
      extra = data.getSerializableExtra(EditLegacyModuleControllerActivity.EDIT_LEGACY_CONFIG);
    }
    if (extra != null) {
      ControllerConfiguration newC = (ControllerConfiguration) extra;
      scannedDevices.put(newC.getSerialNumber(), newC.configTypeToDeviceType(newC.getType()));
      deviceControllers.put(newC.getSerialNumber(), newC);
      populateList();

      String name = preferredFilename;
      // only update the filename if it hasn't already been updated to have "unsaved" in it
      if (!name.toLowerCase().contains(Utility.UNSAVED.toLowerCase())){
        name = Utility.UNSAVED + " " + preferredFilename;
        utility.saveToPreferences(name, R.string.pref_hardware_config_filename);
        preferredFilename = name;
      }

    } else {
      DbgLog.error("Received Result with an incorrect request code: " + String.valueOf(requestCode));
    }

  }

  @Override
  protected void onDestroy(){
    super.onDestroy();
    utility.resetCount();
  }

  /**
   * This has a lot of code in common with saveConfiguration. This is apparently the only way to do
   * this, because of the way the onClickListener accesses the "final EditText input". I
   * modularized as much as I could but this was as far as I got.
   *
   * Note that onBackPressed has to have a finish() once you click the "OK" button or the back button
   * won't actually go back.
   */
  @Override
  public void onBackPressed() {
    if (preferredFilename.toLowerCase().contains(Utility.UNSAVED.toLowerCase())){
      boolean foundDuplicates = utility.writeXML(deviceControllers);
      if (foundDuplicates){
        return;
      }
      String message = "Please save your file before exiting the Hardware Wizard! \n " +
          "If you click 'Cancel' your changes will be lost.";
      final EditText input = new EditText(this);
      String currentFile = utility.prepareFilename(preferredFilename);

      input.setText(currentFile);
      AlertDialog.Builder builder = utility.buildBuilder("Unsaved changes", message);
      builder.setView(input);
      DialogInterface.OnClickListener ok_listener = new DialogInterface.OnClickListener(){
        public void onClick(DialogInterface dialog, int button){
          String filename = input.getText().toString()+Utility.FILE_EXT;
          filename = filename.trim();
          if (filename.equals(".xml")){
            utility.complainToast("File not saved: Please entire a filename.", context);
            return;
          }
          try {
            utility.writeToFile(filename);
          } catch (RobotCoreException e) {
            utility.complainToast(e.getMessage(), context);
            DbgLog.error(e.getMessage());
            return;
          } catch (IOException e){
            warnDuplicateNames(e.getMessage());
            DbgLog.error(e.getMessage());
            return;
          }
          clearDuplicateWarning();
          utility.saveToPreferences(input.getText().toString(), R.string.pref_hardware_config_filename);
          preferredFilename = input.getText().toString();
          utility.updateHeader(Utility.DEFAULT_ROBOT_CONFIG, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
          utility.confirmSave();
          finish();
        }
      };

      builder.setPositiveButton("Ok", ok_listener);
      builder.setNegativeButton("Cancel", back_cancel_listener);
      builder.show();
    } else {
      super.onBackPressed();
    }
  }

  /**
   * The cancel listener that changes the filename and ends the activity, like one would
   * expect the back button to do.
   */
  DialogInterface.OnClickListener back_cancel_listener = new DialogInterface.OnClickListener(){
    public void onClick(DialogInterface dialog, int button){
      // Set preferredFilename to unchanged file that was there previously
      utility.saveToPreferences(preferredFilename.substring(7).trim(), R.string.pref_hardware_config_filename); // chop off "Unsaved"
      finish();
    }
  };

  /**
   * A button-specific method, this gets called when you click the "writeXML" button.
   * This writes the current objects into an XML file located in the Configuration File Directory.
   * The user is prompted for the name of the file.
   * @param v the View from which this was called
   */
  public void saveConfiguration(View v){

    boolean foundDuplicates = utility.writeXML(deviceControllers);
    if (foundDuplicates){
      return;
    }

    String message = "Please enter the file name";
    final EditText input = new EditText(this);
    String currentFile = utility.prepareFilename(preferredFilename);

    input.setText(currentFile);
    AlertDialog.Builder builder = utility.buildBuilder("Enter file name", message);
    builder.setView(input);
    DialogInterface.OnClickListener ok_listener = new DialogInterface.OnClickListener(){
      public void onClick(DialogInterface dialog, int button){
        String filename = input.getText().toString()+Utility.FILE_EXT;
        filename = filename.trim();
        if (filename.equals(".xml")){
          utility.complainToast("File not saved: Please entire a filename.", context);
          return;
        }
        try {
          utility.writeToFile(filename);
        } catch (RobotCoreException e) {
          utility.complainToast(e.getMessage(), context);
          DbgLog.error(e.getMessage());
          return;
        } catch (IOException e){
          //utility.complainToast("Found " + e.getMessage() + "\n Please fix and re-save", context);
          warnDuplicateNames(e.getMessage());
          DbgLog.error(e.getMessage());
          return;
        }
        clearDuplicateWarning();
        utility.saveToPreferences(input.getText().toString(), R.string.pref_hardware_config_filename);
        preferredFilename = input.getText().toString();
        utility.updateHeader(Utility.DEFAULT_ROBOT_CONFIG, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
        utility.confirmSave();
      }
    };

    builder.setPositiveButton("Ok", ok_listener);
    builder.setNegativeButton("Cancel", do_nothing_cancel_listener);
    builder.show();
  }

  DialogInterface.OnClickListener do_nothing_cancel_listener = new DialogInterface.OnClickListener(){
    public void onClick(DialogInterface dialog, int button){
      // do nothing
    }
  };



  /**
   * Turns a list of device Controllers into a hashmap. When reading from an XML file,
   * you get a list back, so this builds up the hashmap from that list. The hashmap gets
   * used to populate the lists displaying the current devices.
   * @param deviceList a list of devices
   */
  private void buildHashMap(ArrayList<ControllerConfiguration> deviceList){
    deviceControllers = new HashMap<SerialNumber, ControllerConfiguration>();

    for (ControllerConfiguration controller : deviceList){
      deviceControllers.put(controller.getSerialNumber(), controller);
    }
  }

}

