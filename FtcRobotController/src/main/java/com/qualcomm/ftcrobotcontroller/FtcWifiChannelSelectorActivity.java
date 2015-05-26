/*
 * Copyright (c) 2014 Qualcomm Technologies Inc
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Qualcomm Technologies Inc nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package com.qualcomm.ftcrobotcontroller;

import android.app.Activity;
import android.app.ProgressDialog;
import android.content.Context;
import android.content.Intent;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemSelectedListener;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.Spinner;
import android.widget.Toast;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.wirelessp2p.WifiDirectChannelSelection;

import java.io.IOException;

public class FtcWifiChannelSelectorActivity extends Activity
    implements OnItemSelectedListener, OnClickListener {

  private static final int INVALID = WifiDirectChannelSelection.INVALID;
  private static int spinnerSelection = 0;

  private Button buttonConfigure;
  private Button buttonWifiSettings;
  private Spinner spinner;
  private ProgressDialog progressDialog;


  private WifiDirectChannelSelection wifiConfig;

  private int wifi_direct_class   = -1;
  private int wifi_direct_channel = -1;

  private Context context;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_ftc_wifi_channel_selector);

    context = this;

    spinner = (Spinner) findViewById(R.id.spinnerChannelSelect);
    ArrayAdapter<CharSequence> adapter =
        ArrayAdapter.createFromResource(this, R.array.wifi_direct_channels,
            android.R.layout.simple_spinner_item);
    adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
    spinner.setAdapter(adapter);
    spinner.setOnItemSelectedListener(this);

    buttonConfigure = (Button) findViewById(R.id.buttonConfigure);
    buttonConfigure.setOnClickListener(this);

    buttonWifiSettings = (Button) findViewById(R.id.buttonWifiSettings);
    buttonWifiSettings.setOnClickListener(this);

    WifiManager wifiManager = (WifiManager) this.getSystemService(Context.WIFI_SERVICE);
    wifiConfig = new WifiDirectChannelSelection(this, wifiManager);
  }

  @Override
  protected void onStart() {
    super.onStart();
    spinner.setSelection(spinnerSelection);
  }

  @Override
  public void onItemSelected(AdapterView<?> av, View v, int item, long l) {
    switch(item) {
      case  0: wifi_direct_class = INVALID; wifi_direct_channel = INVALID; break;

      case  1: wifi_direct_class =  81; wifi_direct_channel =   1; break;
      case  2: wifi_direct_class =  81; wifi_direct_channel =   6; break;
      case  3: wifi_direct_class =  81; wifi_direct_channel =  11; break;
/*
      case  4: wifi_direct_class = 115; wifi_direct_channel =  36; break;
      case  5: wifi_direct_class = 115; wifi_direct_channel =  40; break;
      case  6: wifi_direct_class = 115; wifi_direct_channel =  44; break;
      case  7: wifi_direct_class = 115; wifi_direct_channel =  48; break;
*/
      case  4: wifi_direct_class = 124; wifi_direct_channel = 149; break;
      case  5: wifi_direct_class = 124; wifi_direct_channel = 153; break;
      case  6: wifi_direct_class = 124; wifi_direct_channel = 157; break;
      case  7: wifi_direct_class = 124; wifi_direct_channel = 161; break;
    }
  }

  @Override
  public void onNothingSelected(AdapterView<?> av) {
    // take no action
  }

  @Override
  public void onClick(View v) {
    switch (v.getId()) {
      case R.id.buttonConfigure:
        spinnerSelection = spinner.getSelectedItemPosition();
        configure();
        break;
      case R.id.buttonWifiSettings:
        DbgLog.msg("launch wifi settings");

        startActivity(new Intent(WifiManager.ACTION_PICK_WIFI_NETWORK));
        break;
    }
  }

  private void configure() {
    DbgLog.msg(String.format("configure p2p channel - class %d channel %d",
        wifi_direct_class, wifi_direct_channel));

    try {
      progressDialog = ProgressDialog.show(this, "Configuring Channel", "Please Wait", true);

      wifiConfig.config(wifi_direct_class, wifi_direct_channel);
      new Thread(new Runnable() {
        @Override
        public void run() {
          try {
            Thread.sleep(5 * 1000); // in milliseconds
          } catch (InterruptedException ignored) { } // ignore

          runOnUiThread(new Runnable() {
            @Override
            public void run() {
              setResult(RESULT_OK);
              progressDialog.dismiss();
              finish();
            }
          });
        }
      }).start();
    } catch (IOException e) {
      toast("Failed - root is required", Toast.LENGTH_SHORT);
      e.printStackTrace();
    }
  }

  private void toast(final String msg, final int toastLength) {
    Runnable toast = new Runnable() {
      @Override
      public void run() {
        Toast.makeText(context, msg, toastLength).show();
      }
    };

    runOnUiThread(toast);
  }
}
