/* Copyright (c) 2015 Qualcomm Technologies Inc

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
import android.graphics.Color;
import android.os.Bundle;
import android.os.Environment;
import android.text.Spannable;
import android.text.SpannableString;
import android.text.style.ForegroundColorSpan;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.ScrollView;
import android.widget.TextView;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.RunShellCommand;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.Serializable;


public class ViewLogsActivity extends Activity {

  TextView textAdbLogs;
  int DEFAULT_NUMBER_OF_LINES = 300;
  public static final String FILENAME = "Filename";

  String filepath = " ";

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_view_logs);

    textAdbLogs = (TextView) findViewById(R.id.textAdbLogs);

    final ScrollView scrollView = ((ScrollView) findViewById(R.id.scrollView));
    scrollView.post(new Runnable() {
      @Override
      public void run() {
        scrollView.fullScroll(ScrollView.FOCUS_DOWN);
      }
    });
  }

  @Override
  protected void onStart() {
    super.onStart();

    Intent intent = getIntent();
    Serializable extra = intent.getSerializableExtra(FILENAME);
    if(extra != null) {
      filepath = (String) extra;
    }
    runOnUiThread(new Runnable() {
      @Override
      public void run() {
        try {
          String output = readNLines(DEFAULT_NUMBER_OF_LINES);
          Spannable colorized = colorize(output);
          textAdbLogs.setText(colorized);
        } catch (IOException e) {
          RobotLog.e(e.toString());
          textAdbLogs.setText("File not found: " + filepath);
        }
      }
    });
  }

  public String readNLines(int n) throws IOException {
    File sdcard = Environment.getExternalStorageDirectory();
    File file = new File(filepath);
    BufferedReader bufferedReader = new BufferedReader(new FileReader(file));
    String[] ringBuffer = new String[n];
    int totalLines = 0;
    String line = null;
    // read into the circular buffer, storing only 'n' lines at a time.
    while ((line = bufferedReader.readLine()) != null) {
      ringBuffer[totalLines % ringBuffer.length] = line;
      totalLines++;
    }

    // this may be in the middle of the ringbuffer,
    // so if we mod by the length of the ringBuffer, we'll get the
    // "start" of the lines. i.e., the "oldest" line.
    int start = totalLines - n;
    if (start < 0) {
      start = 0;
    }

    String output = "";
    for (int i = start; i < totalLines; i++) {
      // this will get you to the "oldest" line in the ringBuffer
      int index = i % ringBuffer.length;
      String currentLine = ringBuffer[index];
      output += currentLine + "\n";
    }

    // Logcat sometimes duplicates logs, so we can also just read from
    // the last "--------- beginning" print out.
    int mostRecentIndex = output.lastIndexOf("--------- beginning");
    if (mostRecentIndex < 0) {
      // that string wasn't found, so just return everything
      return output;
    }
    return output.substring(mostRecentIndex);

  }

  private Spannable colorize(String output) {
    Spannable span = new SpannableString(output);
    String[] lines = output.split("\\n");
    int currentStringIndex = 0;
    for (String line : lines) {
      if (line.contains("E/RobotCore") || line.contains(DbgLog.ERROR_PREPEND)) {
        span.setSpan(new ForegroundColorSpan(Color.RED),
            currentStringIndex, currentStringIndex + line.length(),
            Spannable.SPAN_EXCLUSIVE_EXCLUSIVE);
      }
      currentStringIndex += line.length();
      currentStringIndex++; // add for each new line character that we "split" by.
    }

    return span;
  }
}
