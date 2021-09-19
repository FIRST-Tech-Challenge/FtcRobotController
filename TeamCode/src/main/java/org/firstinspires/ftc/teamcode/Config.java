package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.SharedPreferences;

public class Config {

        private static final String PREFERENCES = "BravenatorsConfig";
        private final SharedPreferences sharedPreferences;

        public Config(Context context) {
            sharedPreferences = context.getSharedPreferences(PREFERENCES, Context.MODE_PRIVATE);
            Load();
        }

        public void Load() {
            allianceColor = AllianceColor.ToAllianceColor(sharedPreferences.getString(ALLIANCE_COLOR, "RED"));
            startingPosition = StartingPosition.ToStartingPosition(sharedPreferences.getString(STARTING_POSITION, "STORAGE_UNIT"));
        }

        public void Save() {
            SharedPreferences.Editor editor = sharedPreferences.edit();

            editor.putString(ALLIANCE_COLOR, allianceColor.name());
            editor.putString(STARTING_POSITION, startingPosition.name());

            editor.apply();
        }

        // Alliance Color
        public static final String ALLIANCE_COLOR = "AllianceColor";
        public AllianceColor allianceColor;
        public enum AllianceColor {
            RED, BLUE;

            public static AllianceColor ToAllianceColor(String allianceColor) {
                try {
                    return valueOf(allianceColor);
                } catch(Exception e) {
                    return RED; // Default Alliance Color
                }
            }
        }

        // Starting Position
        public static final String STARTING_POSITION = "StartingPosition";
        public StartingPosition startingPosition;
        public enum StartingPosition {
            STORAGE_UNIT,
            SHIPPING_HUB;

            public static StartingPosition ToStartingPosition(String startingPosition) {
                try {
                    return valueOf(startingPosition);
                } catch (Exception e) {
                    return STORAGE_UNIT;
                }
            }
        }

}
