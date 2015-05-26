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
import android.os.Bundle;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.TextView;

import com.qualcomm.robotcore.util.Version;

/**
 * Basic empty About class. All configurations for
 * About menu is done in the xml file.
 *
 */
//public class About extends PreferenceActivity
public class AboutActivity extends Activity
{
  /** Called when the activity is first created. */
  @Override
  protected void onCreate( Bundle savedInstanceState )
  {
    super.onCreate(savedInstanceState);
    //addPreferencesFromResource( R.layout.about );
    setContentView(R.layout.about);

    ListView aboutList = (ListView)findViewById(R.id.aboutList);

    ArrayAdapter<String[]> adapter =
        new ArrayAdapter<String[]>(this, android.R.layout.simple_list_item_2, android.R.id.text1)
        {
          @Override
          public View getView(int position, View convertView, ViewGroup parent) {
            View view = super.getView(position, convertView, parent);
            TextView topLine = (TextView) view.findViewById(android.R.id.text1);
            TextView bottomLine = (TextView) view.findViewById(android.R.id.text2);

            String[] item = getItem(position);
            if( item.length == 2 ) {
              topLine.setText(item[0]);
              bottomLine.setText(item[1]);
            }

            return view;
          }

          @Override
          public int getCount()
          {
            return 2;
          }

          @Override
          public String[] getItem( int pos )
          {
            String[] rc = new String[2];
            if( pos == 0 ) {
              try
              {
                rc[0] = "App Version";
                rc[1] = AboutActivity.this.getPackageManager()
                    .getPackageInfo(AboutActivity.this.getPackageName(), 0).versionName;
              }
              catch( android.content.pm.PackageManager.NameNotFoundException e ) {
              }
            }
            else if( pos == 1 ) {
              rc[0] = "Build Version";
              rc[1] = Version.getBuildLabel( AboutActivity.this );
            }
            return rc;
          }
        };

        aboutList.setAdapter(adapter);
  }
}

