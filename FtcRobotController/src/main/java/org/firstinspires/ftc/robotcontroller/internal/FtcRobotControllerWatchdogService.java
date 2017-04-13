/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.os.IBinder;
import android.support.annotation.Nullable;

import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * {@link FtcRobotControllerWatchdogService} is a simple sticky service that we use to detect
 * a crash of the robot controller process and auto-restart the robot controller activity when
 * that happens.
 */
@SuppressWarnings("WeakerAccess")
public class FtcRobotControllerWatchdogService extends Service
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public static final String TAG = "FtcRobotControllerWatchdogService";

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    @Nullable @Override public IBinder onBind(Intent intent)
        {
        return null; // we're not this kind of service: we're a 'startable' not a 'bindable' one
        }

    //----------------------------------------------------------------------------------------------
    // Life Cycle
    //----------------------------------------------------------------------------------------------

    @Override public void onCreate()
        {
        super.onCreate();
        RobotLog.vv(TAG, "onCreate()");
        }

    /** On restart after crash, intent is always null; when the RC activity starts us, it's never null */
    @Override public int onStartCommand(Intent intent, int flags, int startId)
        {
        RobotLog.vv(TAG, "onStartCommand() intent=%s flags=0x%x startId=%d", intent, flags, startId);
        boolean autoStart = shouldAutoLaunchRobotController();
        if (null == intent && autoStart)
            {
            launchRobotController(this);
            }
        return autoStart ? START_STICKY : START_NOT_STICKY;
        }

    @Override public void onDestroy()
        {
        super.onDestroy();
        RobotLog.vv(TAG, "onDestroy()");
        }

    //----------------------------------------------------------------------------------------------
    // Operations
    //----------------------------------------------------------------------------------------------

    public static boolean shouldAutoLaunchRobotController()
        {
        boolean result = false;

        // We only *ever* autorun in the embedded, headless lynx case
        if (LynxConstants.isDragonboardWithEmbeddedLynxModule())
            {
            // But we might be asked to pretend we're not there
            if (!LynxConstants.disableDragonboard())
                {
                // We examine the policy flag
                if (LynxConstants.autorunRobotController())
                    {
                    result = true;
                    }
                }
            }

        RobotLog.vv(TAG, "shouldAutoLauchRobotController() result=%s", result);
        return result;
        }

    public static void launchRobotController(Context context)
        {
        RobotLog.vv(TAG, "launchRobotController()");
        Intent openApp = new Intent(context, FtcRobotControllerActivity.class);
        openApp.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);    // nb: task != process
        context.startActivity(openApp);
        }

    }
