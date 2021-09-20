package org.firstinspires.ftc.teamcode.skills;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;

public class SoundEffect {
    boolean soundPlaying = false;
    HardwareMap hardwareMap;

    public SoundEffect(HardwareMap hMap){
        hardwareMap = hMap;
    }


    protected void play(int resourceID){
        if (!soundPlaying) {
            soundPlaying = true;
            MediaPlayer player = MediaPlayer.create(hardwareMap.appContext, resourceID);
            player.setOnCompletionListener(new MediaPlayer.OnCompletionListener() {
                @Override
                public void onCompletion(MediaPlayer mediaPlayer) {
                    soundPlaying = false;
                }
            });
            player.start();
        }
    }
}
