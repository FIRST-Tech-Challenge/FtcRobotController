<?xml version="1.0" encoding="utf-8"?>

<!--

app_settings.xml in FtcRobotController

Copyright (c) 2014, 2015 Qualcomm Technologies Inc

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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

See https://developer.android.com/guide/topics/ui/settings.html
-->

<PreferenceScreen xmlns:tools="http://schemas.android.com/tools"
                  xmlns:android="http://schemas.android.com/apk/res/android"
                  xmlns:app="http://schemas.android.com/apk/res-auto"
                  tools:context=".FtcRobotControllerActivity">

  <PreferenceCategory
    android:title="@string/prefcat_configure_robot">

    <EditTextPreference
        android:title="@string/prefedit_device_name_rc"
        android:summary="@string/prefedit_device_name_summary_rc"
        android:key="@string/pref_device_name"
        android:defaultValue=""
        />

    <org.firstinspires.ftc.robotcore.internal.ui.ColorListPreference
        android:title="@string/prefedit_app_theme_rc"
        android:summary="@string/prefedit_app_theme_summary_rc"
        android:key="@string/pref_app_theme"
        android:entries="@array/app_theme_names"
        android:entryValues="@array/app_theme_tokens"
        app:colors="@array/app_theme_colors"
        />

    <SwitchPreference
        android:title="@string/prefedit_sound_on_off"
        android:summary="@string/prefedit_sound_on_off_summary_rc"
        android:key="@string/pref_sound_on_off"
        android:defaultValue="true"
        />

    <PreferenceScreen
        android:title="@string/titleAdvancedRCSettings"
        android:summary="@string/summaryAdvancedRCSettings"
        android:key="@string/pref_launch_advanced_rc_settings">
        <intent
            android:targetPackage="@string/packageName"
            android:targetClass="com.qualcomm.ftccommon.FtcAdvancedRCSettingsActivity"
            />
    </PreferenceScreen>

    <PreferenceScreen
        android:title="@string/prefedit_view_logs"
        android:summary="@string/prefedit_view_logs_summary"
        android:key="@string/pref_launch_viewlogs">
      <intent
        android:targetPackage="@string/packageName"
        android:targetClass="com.qualcomm.ftccommon.ViewLogsActivity" />
    </PreferenceScreen>

  </PreferenceCategory>

</PreferenceScreen>