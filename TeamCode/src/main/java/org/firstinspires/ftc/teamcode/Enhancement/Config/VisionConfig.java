package org.firstinspires.ftc.teamcode.Enhancement.Config;

import org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Vision.DetectMarker.MarkerLocation;

/**
 * Vision Config
 * WARNING: YOU SHOULD NOT NORMALLY HAVE TO EDIT THIS FILE.
 */
public class VisionConfig extends Config {
    public static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    public static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    public static final int HORIZON = 100; // horizon value to tune

    public static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam
    public static final String VUFORIA_KEY = "ATDGULf/////AAABmRRGSyLSbUY4lPoqBYjklpYqC4y9J7bCk42kjgYS5KtgpKL8FbpEDQTovzZG8thxB01dClvthxkSuSyCkaZi+JiD5Pu0cMVre3gDwRvwRXA7V9kpoYyMIPMVX/yBTGaW8McUaK9UeQUaFSepsTcKjX/itMtcy7nl1k84JChE4i8whbinHWDpaNwb5qcJsXlQwJhE8JE7t8NMxMm31AgzqjVf/7HwprTRfrxjTjVx5v2rp+wgLeeLTE/xk1JnL3fZMG6yyxPHgokWlIYEBZ5gBX+WJfgA+TDsdSPY/MnBp5Z7QxQsO9WJA59o/UzyEo/9BkbvYJZfknZqeoZWrJoN9jk9sivFh0wIPsH+JjZNFsPw"; // TODO: Get new VUFORIA KEY
    public static MarkerLocation finalMarkerLocation = MarkerLocation.SEARCHING;

    @Override
    public Object get(String key) {
        return null;
    }

    @Override
    public void set(String key, Object value) {

    }
}

/* vuforia acc credentials
username: FTC5206
password: Robotics5206
 */
