package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp(name="configure shared prefs", group="TeleOp OpMode")
public class configureSharedPrefsOpMode extends OpMode {


    final String[][] VALUES={//all possible values of the prefs
            {"STRAFE","FORWARD_ONLY"},
            {"0_SECONDS","5_SECONDS","10_SECONDS","15_SECONDS","20_SECONDS"},
            {"THEIR_COLOR","OUR_COLOR"},
            {"BLUE","RED"},
            {"MEDIUM_LARGE","SMALL_LARGE","SMALL_MEDIUM","SMALL_SMALL","LARGE_LARGE"}
    };
    String [] currentValues=new String[VALUES.length];
    final String[] PREF_NAMES ={"movement mode","wait time","starting side","alliance color","auton type"};//human-readable names of the prefs
    int[] currentIndexes =new int[VALUES.length];
    int currSel=0, dispUntil;
    SharedPreferences sharedPrefs;
    boolean prevUP,prevDOWN,prevLEFT,prevRIGHT;
    @Override
    @SuppressWarnings("deprecation")
    public void init() {
        Arrays.fill(currentIndexes, -1);//initialize all values in currentIndexes to -13
        sharedPrefs = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);

        //display the starting info
        telemetry.addData("ready to set shared prefs.\npres play to start","");
        telemetry.update();

        //load in all the current values
        currentValues[0] = sharedPrefs.getString("movement_mode", "ERROR");
        currentValues[1] = sharedPrefs.getString("wait_time", "ERROR");
        currentValues[2] = sharedPrefs.getString("starting_side", "ERROR");
        currentValues[3] = sharedPrefs.getString("alliance_color", "ERROR");
        currentValues[4] = sharedPrefs.getString("auton_type", "ERROR");
        for(int i=0;i<currentValues.length;i++){
            for(int j=0;j<VALUES[i].length;j++){
                if(currentValues[i].equals(VALUES[i][j])){
                    currentIndexes[i]=j;//set the index value to the corresponding index for the value of the pref
                    break;
                }
            }
        }
    }

    @Override
    public void loop() {
        //print the controls
        telemetry.addData("CONTROLS game pad 1:","d-pad UP/DOWN chane Selected preference. LEFT/RIGHT change value. X save current preferences");

        //display all the prefs and their current values
        for(int i=0;i<VALUES.length;i++){
            telemetry.addData(PREF_NAMES[i],padding(16-PREF_NAMES[i].length())+((i==currSel)?"<":" ")+currentValues[i]+((i==currSel)?">":""));
            //                                                  ^^^ if this is the currently selected pref then surround the name in < >
        }
        //display saved if you saved within the last second
        if(System.nanoTime()/1000000< dispUntil){
            telemetry.addData("SAVED","!!!!!!!");
        }

        telemetry.update();

        //save the prefs when x is pressed
        if(gamepad1.x){
            savePrefs();
            telemetry.addData("SAVED","!!!!");
            telemetry.update();
        }
        //move to the previous pref
        if(gamepad1.dpad_up&&!prevUP){
            currSel--;
            if(currSel==-1)
                currSel=VALUES.length-1;
        }
        //move to the next pref
        if(gamepad1.dpad_down&&!prevDOWN){
            currSel++;
            if(currSel==VALUES.length)
                currSel=0;
        }
        //cycle to the previous option for the currently selected pref
        if(gamepad1.dpad_left&&!prevLEFT){
            currentIndexes[currSel]--;
            if(currentIndexes[currSel]<0){
                currentIndexes[currSel]=VALUES[currSel].length-1;
            }
            currentValues[currSel]=VALUES[currSel][currentIndexes[currSel]];
        }
        //cycle to the next option for the current pref
        if(gamepad1.dpad_right&&!prevRIGHT){
            currentIndexes[currSel]++;
            if(currentIndexes[currSel]>=VALUES[currSel].length){
                currentIndexes[currSel]=0;
            }
            currentValues[currSel]=VALUES[currSel][currentIndexes[currSel]];
        }
        //previous button states. ensures that the selection is only moved once per button press
        prevUP=gamepad1.dpad_up;
        prevDOWN=gamepad1.dpad_down;
        prevLEFT=gamepad1.dpad_left;
        prevRIGHT=gamepad1.dpad_right;

    }

    /**saves the prefs to the phone
     *
     */
    void savePrefs() {
        SharedPreferences.Editor editor = sharedPrefs.edit();
        editor.putString("movement_mode", currentValues[0]);
        editor.putString("wait_time", currentValues[1]);
        editor.putString("starting_side", currentValues[2]);
        editor.putString("alliance_color", currentValues[3]);
        editor.putString("auton_type", currentValues[4]);
        editor.apply();
        //display saved for 1 second
        dispUntil =(int)(System.nanoTime()/1000000)+1000;
    }

    String padding(int a){
        StringBuilder p= new StringBuilder();
        if(a>0)
            for(int i=0;i<a;i++){
                p.append(" ");
            }
        return p.toString();
    }

}
