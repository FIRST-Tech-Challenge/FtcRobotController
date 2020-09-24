/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

package com.hfrobots.tnt.corelib.chaosninja;

import android.content.Context;
import android.content.SharedPreferences;
import android.util.Log;

import com.hfrobots.tnt.corelib.control.ChaosNinjaLandingState;

import java.util.concurrent.TimeUnit;

import lombok.AllArgsConstructor;
import lombok.Builder;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

/**
 * Saves chaos ninja settings between OpModes, restarts of phone, etc.
 */
@AllArgsConstructor
public class ChaosConfigSaver {

    private static final String EXPIRES_PREF_NAME = "expires";
    private static final String CHALLENGE_LEVEL_PREF_NAME = "challengeLevel";
    private static final String METRICS_ACTIVATED_PREF_NAME = "metricsActivated";

    private final Context appContext;

    @AllArgsConstructor
    @Builder
    public static class Config {
        public final boolean applyConfiguration;
        public final int challengeLevel;
        public final boolean metricsActivated;
    }

    public Config getSavedConfiguration() {
        SharedPreferences savedPrefs = getSharedPreferences();

        if (savedPrefs != null) {
            long expiryTime = savedPrefs.getLong(EXPIRES_PREF_NAME, Long.MIN_VALUE); // FIXME: Why?

            if (System.currentTimeMillis() < expiryTime) {
                int savedChallengeLevel = savedPrefs.getInt(CHALLENGE_LEVEL_PREF_NAME, 0);
                boolean savedMetricsActivated = savedPrefs.getBoolean(METRICS_ACTIVATED_PREF_NAME, false);

                return Config.builder()
                        .challengeLevel(savedChallengeLevel)
                        .metricsActivated(savedMetricsActivated)
                        .applyConfiguration(false).build();
            } else {
                Log.d(LOG_TAG, "Saved chaos config exists, but has expired");
            }
        }

        return null;
    }

    public Config getSavedOrNewConfiguration(ChaosNinjaLandingState chaosNinja) {

        // Setup from saved data

        Config savedConfig = getSavedConfiguration();

        boolean useChaosController = false;

        boolean metricsActivated = false;
        int challengeLevel = 0;

        if (savedConfig == null && chaosNinja == null) {
            // if chaosNinja doesn't exist, and no saved prefs, make sure no chaos controller
            // gets created, probably deserves a log entry
            Log.d(LOG_TAG, "Chaos - no saved config, no new config, no chaos!");
            useChaosController = false;
        } else if (chaosNinja == null && savedConfig != null) {
            // Restore saved values
            metricsActivated = savedConfig.metricsActivated;
            challengeLevel = savedConfig.challengeLevel;

            Log.d(LOG_TAG, "Chaos - saved config, no new config, welcome to recurring chaos!");
            useChaosController = true;
        } else if (savedConfig != null) {
            // Restore saved values
            metricsActivated = savedConfig.metricsActivated;
            challengeLevel = savedConfig.challengeLevel;

            useChaosController = true;

            // If chaosNinja (via konami) exists, compare, set values, and update saved prefs
            // if necessary

            if (chaosNinja.isSomethingChanged()) {
                boolean chaosConfigHasChangedFromSaved = false;

                if (chaosNinja.isMetricsActivated() != savedConfig.metricsActivated) {
                    metricsActivated = chaosNinja.isMetricsActivated();
                    chaosConfigHasChangedFromSaved = true;
                }

                if (chaosNinja.getChallengeLevel() != savedConfig.challengeLevel) {
                    challengeLevel = chaosNinja.getChallengeLevel();
                    chaosConfigHasChangedFromSaved = true;
                }

                if (chaosConfigHasChangedFromSaved) {
                    saveChaosPrefs(false, metricsActivated, challengeLevel);
                }
            }

            Log.d(LOG_TAG, "Chaos - saved config, and new config, welcome to merged chaos!");
        } else if (chaosNinja != null && savedConfig == null) {
            // If chaosNinja (via konami) exists, but no saved prefs, setup values from chaosNinja
            // save to config and update expiry time

            metricsActivated = chaosNinja.isMetricsActivated();
            challengeLevel = chaosNinja.getChallengeLevel();
            useChaosController = true;

            saveChaosPrefs(true, metricsActivated, challengeLevel);

            Log.d(LOG_TAG, "Chaos - no saved config, and new config, welcome to new chaos!");
        }

        return Config.builder().applyConfiguration(useChaosController)
                .metricsActivated(metricsActivated)
                .challengeLevel(challengeLevel).build();
    }


    private SharedPreferences getSharedPreferences() {
        SharedPreferences savedPrefs = null;

        savedPrefs = appContext.getSharedPreferences("chaosNinja", Context.MODE_PRIVATE);

        return savedPrefs;
    }

    private void saveChaosPrefs(boolean updateExpiry,
                                       boolean metricsActivated,
                                       int challengeLevel) {
        SharedPreferences savedPrefs = getSharedPreferences();

        if (savedPrefs != null) {
            SharedPreferences.Editor prefsEditor = savedPrefs.edit();

            if (updateExpiry) {
                prefsEditor.putLong(EXPIRES_PREF_NAME, System.currentTimeMillis() + TimeUnit.HOURS.toMillis(3));
            }

            prefsEditor.putInt(CHALLENGE_LEVEL_PREF_NAME, challengeLevel);
            prefsEditor.putBoolean(METRICS_ACTIVATED_PREF_NAME, metricsActivated);
            prefsEditor.apply();
            prefsEditor.commit();
        }
    }
}
