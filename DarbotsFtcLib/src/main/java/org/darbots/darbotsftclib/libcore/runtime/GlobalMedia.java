package org.darbots.darbotsftclib.libcore.runtime;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;

import java.io.File;

public class GlobalMedia {
    public static SoundPlayer globalSoundPlayer = SoundPlayer.getInstance();

    public static void preloadResourceFile(Context context, int resourceID){
        globalSoundPlayer.preload(context,resourceID);
    }
    public static void preloadLocalFile(Context context, File soundFile){
        globalSoundPlayer.preload(context,soundFile);
    }
    /**
     * Play a resource file in an Android Module
     * @param context the context (module) pointer, you can use OpMode.appContext as the TeamCode Module pointer
     * @param waitForPreviousSound whether to wait for any currently-playing non-looping sound to finish before playing
     * @param resourceID the ID of the resource. You can use R.raw.ResourceFileName to get the ID of the resource
     * @param rate the speed the sound is played [0.5 - 2.0]
     * @param volume the volume scaling that will be applied to the sound
     * @param loopSound are we looping the music
     * @param loopTime number of time the sound gets played, -1 = forever.
     */
    public static void playResourceFile(Context context, boolean waitForPreviousSound, int resourceID, float rate, float volume, boolean loopSound, int loopTime){
        int realLoopTime = loopSound ? (loopTime == -1 ? -1 : loopTime - 1) : 0;
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        params.loopControl = realLoopTime;
        params.volume = volume;
        params.waitForNonLoopingSoundsToFinish = waitForPreviousSound;
        params.rate = rate;
        globalSoundPlayer.startPlaying(context,resourceID,params,null,null);
    }

    /**
     * Play a resource file in an Android Module
     * @param context the context (module) pointer, you can use OpMode.appContext as the TeamCode Module pointer
     * @param waitForPreviousSound whether to wait for any currently-playing non-looping sound to finish before playing
     * @param soundFile The file pointer pointing to the sound file
     * @param rate the speed the sound is played [0.5 - 2.0]
     * @param volume the volume scaling that will be applied to the sound
     * @param loopSound are we looping the music
     * @param loopTime number of time the sound gets played, -1 = forever.
     */
    public static void playLocalFile(Context context, boolean waitForPreviousSound, File soundFile, float rate, float volume, boolean loopSound, int loopTime){
        int realLoopTime = loopSound ? (loopTime == -1 ? -1 : loopTime - 1) : 0;
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        params.loopControl = realLoopTime;
        params.volume = volume;
        params.waitForNonLoopingSoundsToFinish = waitForPreviousSound;
        params.rate = rate;
        globalSoundPlayer.startPlaying(context,soundFile,params,null,null);
    }

    public static void stopPlayingAllSounds(){
        globalSoundPlayer.stopPlayingAll();
    }

    public static void stopPlayingLoopSounds(){
        globalSoundPlayer.stopPlayingLoops();
    }
}
