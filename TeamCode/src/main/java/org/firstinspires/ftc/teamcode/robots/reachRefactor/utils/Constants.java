package org.firstinspires.ftc.teamcode.robots.reachRefactor.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.ejml.simple.SimpleMatrix;

import java.util.Arrays;

@Config(value = "FFConstants")
public class Constants {

    //----------------------------------------------------------------------------------------------
    // Control Constants
    //----------------------------------------------------------------------------------------------

    public static double EPSILON = 0.001; // small value used for the approximately equal calculation in MathUtils
    public static double TRIGGER_DEADZONE = 0.2; // gamepad trigger values below this threshold will be ignored

    //----------------------------------------------------------------------------------------------
    // Physical Constants
    //----------------------------------------------------------------------------------------------

    // distance measurements
    public static double MIN_CHASSIS_LENGTH = 0.3;
    public static double MAX_CHASSIS_LENGTH = 0.9;
    public static double WHEEL_RADIUS = 0.1016;
    public static double TRACK_WIDTH = 0.308162;

    // conversion factors
     public static double INCHES_PER_METER = 39.3701;

    //----------------------------------------------------------------------------------------------
    // Enums
    //----------------------------------------------------------------------------------------------

    // enum to store possible game states for use in FF_6832
    public enum GameState {
        TELE_OP("Tele-Op"),
        AUTONOMOUS("Autonomous"),
        AUTONOMOUS_DIAGNOSTIC("Autonomous Diagnostic"),
        MANUAL_DIAGNOSTIC("Manual Diagnostic");

        private String name;

        GameState(String name) {
            this.name = name;
        }

        public String getName() { return name; }

        public static GameState getGameState(int index) {
            return GameState.values()[index];
        }

        public static int getNumGameStates() {
            return GameState.values().length;
        }

        public static int indexOf(GameState gameState) {
            return Arrays.asList(GameState.values()).indexOf(gameState);
        }
    }

    public enum Alliance {
        RED(-1), BLUE(1);

        private int mod;

        Alliance(int mod) {
            this.mod = mod;
        }

        public int getMod() {
            return mod;
        }
    }

    public enum Position {
        START_RED(new SimpleMatrix(new double[][] {{0, 0, 0}})),
        START_BLUE(new SimpleMatrix(new double[][] {{0, 0, 0}}));

        private SimpleMatrix pose;

        Position(SimpleMatrix pose) {
            this.pose = pose;
        }

        public SimpleMatrix getPose() {
            return pose;
        }
    }
}
