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
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.configuration.Utility;

import java.io.File;
import java.util.ArrayList;

public class FtcLoadFileActivity extends Activity {

  private ArrayList<String> fileList = new ArrayList<String>();
  private Context context;
  private Utility utility;


  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_load);

    context = this;
    utility = new Utility(this);
    utility.createConfigFolder();
    buildInfoButtons();
  }

  @Override
  protected void onStart() {
    super.onStart();

    fileList = utility.getXMLFiles();
    warnIfNoFiles();
    utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
    populate();
  }

  private void buildInfoButtons(){
    Button saveConfigButton = (Button) findViewById(R.id.files_holder).findViewById(R.id.info_btn);
    saveConfigButton.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View view) {
        AlertDialog.Builder builder = utility.buildBuilder("Available files", "These are the" +
            " files the Hardware Wizard was able to find. You can edit each file by clicking" +
            " the edit button. The 'Activate' button will set that file as the current" +
            " configuration file, which will be used to start the robot.");
        builder.setPositiveButton("Ok", close_ok_listener);
        AlertDialog alert = builder.create();
        alert.show();
        TextView textView = (TextView) alert.findViewById(android.R.id.message);
        textView.setTextSize(14);
      }
    });

    Button autoConfigButton = (Button) findViewById(R.id.autoconfigure_holder).findViewById(R.id.info_btn);
    autoConfigButton.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View view) {
        AlertDialog.Builder builder = utility.buildBuilder("AutoConfigure", "This is the fastest" +
            " way to get a new machine up and running. The AutoConfigure" +
            " tool will automatically enter some default names for devices. AutoConfigure" +
            " expects at least one Servo Controller, with a servo in port 1 and 2, and at least" +
            " one Motor Controller, with a motor in port 1 and 2. If there are other devices " +
            "attached, the AutoConfigure tool will not name them.");
        builder.setPositiveButton("Ok", close_ok_listener);
        AlertDialog alert = builder.create();
        alert.show();
        TextView textView = (TextView) alert.findViewById(android.R.id.message);
        textView.setTextSize(14);
      }
    });
  }

  DialogInterface.OnClickListener close_ok_listener = new DialogInterface.OnClickListener(){
    public void onClick(DialogInterface dialog, int button){
      //do nothing
    }
  };

  private void warnIfNoFiles() {
    if (fileList.size() == 0) {
      String msg0 = "No files found!";
      String msg1 = "In order to proceed, you must create a new file";
      utility.setOrangeText(msg0, msg1, R.id.empty_filelist, R.layout.orange_warning, R.id.orangetext0, R.id.orangetext1);
    } else {
      ViewGroup empty_filelist = (ViewGroup) findViewById(R.id.empty_filelist);
      empty_filelist.removeAllViews();
      empty_filelist.setVisibility(View.GONE);
    }
  }

  private void populate(){
    ViewGroup inclusionViewGroup = (ViewGroup)findViewById(R.id.inclusionlayout);
    inclusionViewGroup.removeAllViews();
    for (String filename: fileList){
      View child = LayoutInflater.from(this).inflate(R.layout.file_info, null);
      inclusionViewGroup.addView(child);
      TextView name = (TextView) child.findViewById(R.id.filename_editText);
      name.setText(filename);
    }
  }

  public void new_button(View v){
    utility.saveToPreferences(Utility.NO_FILE, R.string.pref_hardware_config_filename);
    Intent intent = new Intent(context, FtcConfigurationActivity.class);
    startActivity(intent);
  }

  public void file_edit_button(View v){
    String filename = getFileName(v, true);
    utility.saveToPreferences(filename, R.string.pref_hardware_config_filename);

    Intent intent = new Intent(context, FtcConfigurationActivity.class);
    startActivity(intent);
  }

  public void file_activate_button(View v){
    String filename = getFileName(v, false);
    utility.saveToPreferences(filename, R.string.pref_hardware_config_filename);
    utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
  }

  public void file_delete_button(View v){
    String filenameWExt = getFileName(v, true);
    File file = new File(Utility.CONFIG_FILES_DIR + filenameWExt);

    if (file.exists()){
      file.delete();
    } else {
      utility.complainToast("That file does not exist: " + filenameWExt, context);
      DbgLog.error("Tried to delete a file that does not exist: " + filenameWExt);
    }

    fileList = utility.getXMLFiles();
    utility.saveToPreferences(Utility.NO_FILE, R.string.pref_hardware_config_filename);
    utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
    populate();
  }

  private String getFileName(View v, boolean plusExtension){
    LinearLayout horizontalButtons = (LinearLayout) v.getParent();
    LinearLayout linearLayout = (LinearLayout) horizontalButtons.getParent();
    TextView name = (TextView) linearLayout.findViewById(R.id.filename_editText);
    String filename = name.getText().toString();
    if (plusExtension) {
      filename += Utility.FILE_EXT;
    }
    return filename;
  }

  public void launch_autoConfigure(View v){
    startActivity(new Intent(getBaseContext(), AutoConfigureActivity.class));
  }

}
