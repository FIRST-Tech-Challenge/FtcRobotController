/*
 * Copyright (c) 2014, 2015 Qualcomm Technologies Inc
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
import android.net.wifi.WifiManager;
import android.os.Bundle;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.wifi.FixWifiDirectSetup;

/**
 * This activity is used to correct any problems detected with the current
 * Wifi Direct settings.
 */
public class ConfigWifiDirectActivity extends Activity {

  private ProgressDialog progressDialog;
  private Context context;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_config_wifi_direct);

    context = this;
  }

  @Override
  protected void onResume() {
    super.onResume();

    (new Thread(new ToggleWifiRunnable())).start();
  }

  /*
   * This runnable toggles wifi off and on, which is the only reliable way to fix a misconfigured
   * wifi direct connection.
   */
  private class ToggleWifiRunnable implements Runnable {
    @Override
    public void run() {
      DbgLog.msg("attempting to reconfigure Wifi Direct");

      runOnUiThread(new Runnable() {
        @Override
        public void run() {
          progressDialog = ProgressDialog.show(context, "Configuring Wifi Direct", "Please Wait", true);
        }
      });

      WifiManager wifiManager = (WifiManager) getSystemService(Context.WIFI_SERVICE);

      try {
        FixWifiDirectSetup.fixWifiDirectSetup(wifiManager);
      } catch (InterruptedException e) {
        DbgLog.msg("Cannot fix wifi setup - interrupted");
      }

      runOnUiThread(new Runnable() {
        @Override
        public void run() {
          progressDialog.dismiss();
          finish();
        }
      });

    }
  }
}
