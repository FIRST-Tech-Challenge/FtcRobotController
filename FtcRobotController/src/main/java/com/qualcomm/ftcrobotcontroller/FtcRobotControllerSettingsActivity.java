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
import android.content.Intent;
import android.os.Build;
import android.os.Bundle;
import android.preference.Preference;
import android.preference.PreferenceFragment;
import android.provider.Settings;
import android.widget.Toast;

import com.qualcomm.ftccommon.Device;

public class FtcRobotControllerSettingsActivity extends Activity {

  public static class SettingsFragment extends PreferenceFragment {
    @Override
    public void onCreate(Bundle savedInstanceState) {
      super.onCreate(savedInstanceState);

      // Load the preferences from an XML resource
      addPreferencesFromResource(R.xml.preferences);


      if (Build.MANUFACTURER.equalsIgnoreCase(Device.MANUFACTURER_ZTE) && Build.MODEL.equalsIgnoreCase(Device.MODEL_ZTE_SPEED)) {
        Preference launchSettings =  findPreference(getString(R.string.pref_launch_settings));
        launchSettings.setOnPreferenceClickListener(new Preference.OnPreferenceClickListener() {

          public boolean onPreferenceClick(Preference preference) {
            Intent intent = getActivity().getPackageManager().getLaunchIntentForPackage("com.zte.wifichanneleditor");
            try {
              startActivity(intent);
            } catch (NullPointerException e) {
              Toast.makeText(getActivity(), "Unable to launch ZTE WifiChannelEditor", Toast.LENGTH_SHORT).show();
            }
            return true;
          }
        });
      }

      if (Build.MODEL.equals(Device.MODEL_FOXDA_FL7007)) {
        Preference launchSettings =  findPreference(getString(R.string.pref_launch_settings));
        launchSettings.setOnPreferenceClickListener(new Preference.OnPreferenceClickListener() {

          public boolean onPreferenceClick(Preference preference) {
            Intent viewIntent = new Intent(Settings.ACTION_SETTINGS);
            startActivity(viewIntent);

            return true;
          }
        });
      }
    }
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    // Display the fragment as the main content.
    getFragmentManager().beginTransaction()
    .replace(android.R.id.content, new SettingsFragment())
    .commit();
  }
}
