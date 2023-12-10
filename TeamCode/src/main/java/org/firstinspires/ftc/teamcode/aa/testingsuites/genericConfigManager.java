package org.firstinspires.ftc.teamcode.aa.testingsuites;

import java.util.HashMap;

public class genericConfigManager {
    private HashMap<String, Integer> reserveScheme1;

    public static int schemeNumber = 0;
    protected HashMap<String, String> centralScheme;

    public static HashMap<String, String> memLeakedSceheme;
    public HashMap<String, Object> scheme1;
    public HashMap<String, Object> scheme2;
    public HashMap<String, Object> scheme3;
    public HashMap<String, Object> scheme4;
    public HashMap<String, Object> scheme5;
    public HashMap<String, Object> scheme6;

    private HashMap<String,Integer> noChange;

    public void resetConfiguration() {

    }

    public static schemaThing staticSchemaThing = new schemaThing();

    public void setConfig(HashMap<String, Object> config) {
        // TODO:Fix this method;
    }
    public void setConfigCentral(HashMap<String, String> config) {
        centralScheme = config;
    }
    public void setConfig(HashMap<String, Object> config, String schemeNumber) throws Exception{
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
    public HashMap<String, Object> getScheme(String schemeName) {
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

    public schemaThing reserveScheme() throws Exception {
        String schemeName = "noScheme";
        if (schemeNumber == 0) {
            schemeNumber++;
            schemeName = "scheme1";
        } else if (schemeNumber == 1) {
            schemeNumber++;
            schemeName = "scheme2";
        }else if (schemeNumber == 2) {
            schemeNumber++;
            schemeName = "scheme3";
        }else if (schemeNumber == 3) {
            schemeNumber++;
            schemeName = "scheme4";
        }else if (schemeNumber == 4) {
            schemeNumber++;
            schemeName = "scheme5";
        }else if (schemeNumber == 5) {
            schemeNumber++;
            schemeName = "scheme6";
        }else if (schemeNumber == 0) {
            throw new Exception("no extra schemes present");
        }
        return new schemaThing(schemeName, schemeNumber);
    }
    public static String getSchemeName(schemaThing schema) {
        return schema.schemeName;
    }
    public static int getSchemeNumber(schemaThing schema) {
        return schema.schemeNumber;
    }
}
class schemaThing {
    public String schemeName;
    public int schemeNumber;
    public schemaThing(String name, int number) {
        this.schemeName = name;
        this.schemeNumber = number;
    }
    /** This method is reserved ONLY for the internal static methods systems. */
    public schemaThing() {

    }

}