package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.FTCPanels;

@TeleOp()
 @Disabled
public class FTCPanelTestOpMode extends OpMode {
    public byte RedAlliance = 0x00;
    public byte BlueAlliance = 0x01;

    private FTCPanels panels;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private int colorIndex = 0;
    private @ColorInt int[] colors = new int[] { Color.rgb(148, 0, 211), Color.rgb(75, 0, 130), Color.rgb(0, 0, 255),
            Color.rgb(0, 255, 0), Color.rgb(255, 0, 0), Color.rgb(255, 255, 0), Color.parseColor("purple"),
            Color.parseColor("teal"), Color.parseColor("silver"), Color.rgb(0, 0, 0) };

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        panels = hardwareMap.get(FTCPanels.class, "ftc_panels");
        panels.setAlliance(RedAlliance);
        panels.setBrightness(4);
        panels.setColor(Color.parseColor("red"));
        panels.setColor(5, Color.parseColor("blue"));

    }

    @Override
    public void start() {
        elapsedTime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        if (elapsedTime.milliseconds() >= 500) {
            if (colorIndex == colors.length) {
                panels.setColors(colors);
                colorIndex = 0;
            } else {
                panels.setColor(colors[colorIndex]);
                colorIndex++;
            }
            elapsedTime.reset();
        }
    }

    @Override
    public void stop() {
        panels.turnAllOff();
    }
}