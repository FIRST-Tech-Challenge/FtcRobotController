package org.firstinspires.ftc.teamcode.dcs15815.DefenderFramework.DefenderBot;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

abstract public class DefenderBotSystem {
    public HardwareMap hardwareMap;
    public DefenderBotConfiguration BuzzBotConfiguration;
    public DefenderBot bot;
    private boolean logging = false;

    // Systems are generally responsible for their own setup. To make that happen, this abstract
    // parent class saves references to the hardwareMap provided by the opmode and a reference to
    // the bot itself (useful for things like accessing telemetry). It also used to keep a copy
    // of the configuration file, but since then we've moved to static configuration properties
    // and not instantiated configuration objects
    public DefenderBotSystem(HardwareMap hm, DefenderBot b) {
        this.hardwareMap = hm;
        this.bot = b;
    }

    public boolean isLogging() {
        return logging;
    }

    public void setLogging(boolean logging) {
        this.logging = logging;
    }

    // A convenience method that can be called from any subsystem. Removes the need
    // to have try/catch blocks muddying up code.
    public void sleep(long milliseconds) {
        bot.sleep(milliseconds);
    }

}