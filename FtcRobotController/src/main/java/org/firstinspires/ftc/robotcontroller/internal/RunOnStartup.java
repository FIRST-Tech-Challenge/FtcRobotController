/*Copyright (c) 2016, Justin Niezrecki

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of NAME nor the names of its contributors may be used to
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
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
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
*/

package org.firstinspires.ftc.robotcontroller.internal;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.robotcore.internal.DragonboardLynxDragonboardIsPresentPin;
import org.firstinspires.ftc.robotcore.internal.PreferencesHelper;

/**
 * {@link RunOnStartup} is responsible for auto-starting the robot controller app when
 * a headless device boots.
 */
public class RunOnStartup extends BroadcastReceiver
    {
    public static final String TAG = "RunOnStartup";

    protected Context           context = null;
    protected PreferencesHelper preferencesHelper = null;

    @Override public void onReceive(Context context, Intent intent)
        {
        this.context = context;
        if (preferencesHelper == null)
            {
            preferencesHelper = new PreferencesHelper(TAG, context);
            }

        String action = intent.getAction();
        RobotLog.vv(TAG, "onReceive() action=%s", action);

        // Note: we will be *awakened if necessary* for each of these notifications since we're
        // signed up to see them in our manifest. However, we *also* receive them even if the
        // robot controller is executed manually. In the latter case, we especially don't want
        // to inadvertently terminate this process :-).

        if (action.equals(Intent.ACTION_BOOT_COMPLETED))
            {
            // On boot, we initialize the state that will help us auto-launch the robot
            // controller *exactly* once. Note that we're taking advantage here of the fact
            // that (a) we only ever receive ACTION_BOOT_COMPLETED at most one time, no matter
            // how many times this BroadcastReceiver gets restarted, and (b) we receive
            // ACTION_BOOT_COMPLETED before we receive other notifications that we might be
            // interested in (note that currently *there*are*no* such notifications: we auto-launch
            // here)
            preferencesHelper.remove(context.getString(R.string.pref_autostarted_robot_controller));

            if (shouldAutoLaunchRobotController())
                {
                launchRobotController();
                }
            else
                {
                // Having cleared that state, if we *still* shouldn't launch the robot
                // controller, then we've got no business sticking around.
                noteDragonboardPresenceAndExitIfNoRC();
                }
            }
        else
            {
            noteDragonboardPresenceAndExitIfNoRC();
            }
        }

    /** If we're on the DB/Lynx comb, then ensure that the 'isPresent' pin is in the appropriate
     * state. Then, unless the RC is running in *this* incarnation of this broadcast receiver, get
     * the heck out of Dodge. */
    protected void noteDragonboardPresenceAndExitIfNoRC()
        {
        RobotLog.vv(TAG, "noteDragonboardPresenceAndExitIfNoRC()");
        //
        if (LynxConstants.isDragonboardWithEmbeddedLynxModule())
            {
            DragonboardLynxDragonboardIsPresentPin.getInstance().setState(!LynxConstants.disableDragonboard());
            }
        //
        if (!isRobotControllerRunningInThisProcess())
            {
            AppUtil.getInstance().exitApplication();
            }
        }

    protected boolean isRobotControllerRunningInThisProcess()
        {
        // If only this BroadcastReceiver has been executed, not the RC itself, then the root activity will be null
        return AppUtil.getInstance().getRootActivity() instanceof FtcRobotControllerActivity;
        }

    protected boolean shouldAutoLaunchRobotController()
        {
        boolean result = FtcRobotControllerWatchdogService.shouldAutoLaunchRobotController();
        if (result)
            {
            // Finally, we avoid auto-starting more than once (paranoia)
            result = !preferencesHelper.readBoolean(context.getString(R.string.pref_autostarted_robot_controller), false);
            }

        RobotLog.vv(TAG, "shouldAutoLauchRobotController() result=%s", result);
        return result;
        }

    protected void launchRobotController()
        {
        RobotLog.vv(TAG, "launchRobotController()");

        // Start the guy
        FtcRobotControllerWatchdogService.launchRobotController(context);

        // Remember that we did so so that we don't try to do that a second time
        preferencesHelper.writeBooleanPrefIfDifferent(context.getString(R.string.pref_autostarted_robot_controller), true);
        }
    }
