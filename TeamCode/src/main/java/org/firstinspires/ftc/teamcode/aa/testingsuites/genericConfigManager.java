package org.firstinspires.ftc.teamcode.aa.testingsuites;

import java.util.HashMap;

public class genericConfigManager {
    public HashMap<String, Integer> reserveScheme1;
    protected HashMap<String, String> centralScheme;
    public HashMap<String, Integer> scheme1;
    public HashMap<String, Integer> scheme2;
    public HashMap<String, Integer> scheme3;
    public HashMap<String, Integer> scheme4;
    public HashMap<String, Integer> scheme5;
    public HashMap<String, Integer> scheme6;

    private HashMap<String,Integer> noChange;

    public void resetConfiguration() {

    }

    public void setConfig(HashMap<String, String> config) {
        centralScheme = config;
    }
    public void setConfig(HashMap<String, Integer> config, String schemeNumber) throws Exception{
        if (schemeNumber == "1") {
            scheme1 = config;
        } else if (schemeNumber == "2") {
            scheme2 = config;
        } else if (schemeNumber == "3") {
            scheme3 = config;
        } else if (schemeNumber == "4") {
            scheme4 = config;
        } else if (schemeNumber == "5") {
            scheme5 = config;
        } else if (schemeNumber == "6") {
            scheme6 = config;
        } else {
            throw new Exception("invalid scheme");
        }
    }

    public String getConfig(String key) {
        return null;
    }

    public String addConfigData(String key, String data) {
        return null;
    }

    public void replaceConfigKey(String key, String data) throws Exception {

    }
    public HashMap<String, Integer> getScheme( String schemeName) {
        if (schemeName == "scheme1") {
            return scheme1;
        } else if (schemeName == "scheme2") {
            return scheme2;
        } else if (schemeName == "scheme3") {
            return scheme3;
        }else if (schemeName == "scheme4") {
            return scheme4;
        }else if (schemeName == "scheme5") {
            return scheme5;
        }else {
            return scheme6;
        }
    }
}
