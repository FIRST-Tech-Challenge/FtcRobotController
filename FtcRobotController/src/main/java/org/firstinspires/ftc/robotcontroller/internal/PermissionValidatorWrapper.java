/*
 * Copyright (c) 2018 Craig MacFarlane
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
 * Neither the name of Craig MacFarlane nor the names of its contributors may be used to
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
package org.firstinspires.ftc.robotcontroller.internal;

import android.Manifest;
import android.os.Bundle;

import com.qualcomm.ftcrobotcontroller.R;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.robotcore.internal.system.PermissionValidatorActivity;

import java.util.ArrayList;
import java.util.List;

public class PermissionValidatorWrapper extends PermissionValidatorActivity {

    private final String TAG = "PermissionValidatorWrapper";

    /*
     * The list of dangerous permissions the robot controller needs.
     */
    protected List<String> robotControllerPermissions = new ArrayList<String>() {{
        add(Manifest.permission.WRITE_EXTERNAL_STORAGE);
        add(Manifest.permission.READ_EXTERNAL_STORAGE);
        add(Manifest.permission.CAMERA);
        add(Manifest.permission.ACCESS_COARSE_LOCATION);
    }};

    private final static Class startApplication = FtcRobotControllerActivity.class;

    public String mapPermissionToExplanation(final String permission) {
        if (permission.equals(Manifest.permission.WRITE_EXTERNAL_STORAGE)) {
            return Misc.formatForUser(R.string.permRcWriteExternalStorageExplain);
        } else if (permission.equals(Manifest.permission.READ_EXTERNAL_STORAGE)) {
            return Misc.formatForUser(R.string.permRcReadExternalStorageExplain);
        } else if (permission.equals(Manifest.permission.CAMERA)) {
            return Misc.formatForUser(R.string.permRcCameraExplain);
        } else if (permission.equals(Manifest.permission.ACCESS_COARSE_LOCATION)) {
            return Misc.formatForUser(R.string.permAccessCoarseLocationExplain);
        }
        return Misc.formatForUser(R.string.permGenericExplain);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);

        permissions = robotControllerPermissions;
    }

    protected Class onStartApplication()
    {
        FtcRobotControllerActivity.setPermissionsValidated();
        return startApplication;
    }
}
