package org.firstinspires.ftc.teamcode.settings;

import android.content.res.Resources;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;

/**
 * Link between the physical and the software
 */

public class DeviceNames {

    public String[] LEFT_DRIVE;
    public String[] RIGHT_DRIVE;
    public String[] CENTER_DRIVE;

    public DeviceNames(HardwareMap hardwareMap) {
        Resources resources = hardwareMap.appContext.getResources();

        LEFT_DRIVE = resources.getStringArray(R.array.LEFT_DRIVE);
        CENTER_DRIVE = resources.getStringArray(R.array.CENTER_DRIVE);
        RIGHT_DRIVE = resources.getStringArray(R.array.RIGHT_DRIVE);

    }
}
