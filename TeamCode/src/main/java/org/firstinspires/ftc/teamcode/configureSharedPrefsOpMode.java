package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.ArraySet;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;
import java.util.HashSet;

@TeleOp(name="configure shared prefs", group="TeleOp OpMode")
public class configureSharedPrefsOpMode extends OpMode {
    @Override
    public void init() {
        SharedPreferences.Editor editor = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext).edit();
        editor.putString("movement_mode", "STRAFE");
        editor.putString("wait_time","0_SECONDS");
        editor.putString("auton_type","MEDIUM_LARGE");
        editor.putString("starting_side", "OUR_COLOR");
        editor.putString("alliance_color","BLUE");
        editor.apply();
    }

    @Override
    public void loop() {

    }
}
