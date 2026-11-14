package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.ArrayList;
import java.util.List;

/*
 * Power Monitor Subsystem
 * A BLACK BOX for the battery. It only READS. It never moves a motor and has no gamepad code.
 *
 * WHY IT EXISTS: sometimes something on the robot blinks in the middle of a match (the Pinpoint
 * stops saying READY, the camera restarts, the drive drops out of field-centric). We THINK it is
 * a brown-out (the battery voltage dipping when the motors pull a lot of current), but until now
 * nothing wrote down what the battery was doing at that moment. This class does. Then the next
 * time something blinks we can look at the voltage and SEE if it dipped, instead of guessing.
 *
 * WHAT IT READS (from each REV hub, the Control Hub and the Expansion Hub):
 *   - the battery voltage at the hub's power input (getInputVoltage)
 *   - the current the hub's motors are pulling, in amps (getCurrent)
 * Two hubs on one battery should read nearly the same voltage. If one hub always reads lower, the
 * wire, connector or switch feeding THAT hub is losing power (a bad crimp, a loose connector).
 *
 * WHAT IT KEEPS: the lowest voltage seen since START, the highest current seen, and a count of
 * DIPS. A DIP starts when a hub's voltage falls below DIP_THRESHOLD_VOLTS and ends when it climbs
 * back above DIP_THRESHOLD_VOLTS + DIP_END_MARGIN_VOLTS. Every dip is also written to the Robot
 * Controller log (BiobuzzLog, tag BIOBUZZ) with the time, so you can read it after the match even
 * if nobody took a photo of the screen. (How to read the log: see BiobuzzLog.)
 *
 * IT CHANGES NOTHING THE ROBOT DOES. Nothing reads these numbers to make a decision.
 *
 * COST: every voltage or current reading is a short question to the hub (a few milliseconds), so
 * update() only asks every SAMPLE_INTERVAL_SECONDS, not every loop. The "Loop time" line on the
 * screen will show if it ever slows the loop down.
 *
 * The OpMode must call update() ONCE per loop(). Nothing calls it for us.
 *
 * Robot configuration: none needed. The hubs are found automatically.
 */
public class PowerMonitorSubsystem {

    // ------------------------------------------------------------------
    // Settings (starting guesses: tune them from real match data)
    // ------------------------------------------------------------------
    // How often to ask the hubs, in seconds. Smaller catches shorter dips but costs more loop
    // time. 0.1 is ten times per second. A dip shorter than this can be missed, so if a blink
    // happens and this class saw NO dip, the dip may have been very short: try 0.05.
    private static final double SAMPLE_INTERVAL_SECONDS = 0.10;

    // A hub reading below this many volts counts as a DIP. A healthy battery at rest reads
    // about 13 V. TODO (issue #88): Drive a normal match first and look at the lowest voltage the screen
    // shows. Then set this a little BELOW the lowest "normal" number, so only real sags count.
    private static final double DIP_THRESHOLD_VOLTS = 10.0;

    // A dip only ends once the voltage is this far ABOVE the threshold. It stops one noisy
    // reading from counting as two separate dips.
    private static final double DIP_END_MARGIN_VOLTS = 0.3;

    // A reading at or below this is not real (the hub did not answer). It is skipped, so it
    // cannot count as a dip.
    private static final double MIN_REAL_VOLTAGE_VOLTS = 1.0;

    // true = also read each hub's motor current. false = voltage only (a little cheaper).
    // The current helps show WHICH hub is working hard when the voltage dips.
    private static final boolean READ_MOTOR_CURRENT = true;

    // ------------------------------------------------------------------
    // The hubs and the newest answers. These lists are in the same order: slot 0 is the first hub.
    // ------------------------------------------------------------------
    private final List<LynxModule> hubs = new ArrayList<LynxModule>();
    private final List<String> hubNames = new ArrayList<String>();
    private final List<Double> voltagesNow = new ArrayList<Double>();
    private final List<Double> lowestVoltages = new ArrayList<Double>();
    private final List<Double> currentsNow = new ArrayList<Double>();
    private final List<Double> highestCurrents = new ArrayList<Double>();

    private final ElapsedTime sampleTimer = new ElapsedTime();
    private final ElapsedTime sinceStartTimer = new ElapsedTime();

    // The dip log.
    private boolean inDip = false;
    private final ElapsedTime dipTimer = new ElapsedTime();
    private double lowestInThisDip = 99.0;
    private int dipCount = 0;
    private double lowestEverVolts = 99.0;
    private String lastDipText = "none";

    // ==================================================================
    // SETUP
    // ==================================================================

    /*
     * Finds every REV hub in the robot configuration. If there are none (it should not happen on
     * a real robot), everything below quietly does nothing.
     */
    public PowerMonitorSubsystem(HardwareMap hardwareMap) {
        List<LynxModule> foundHubs = hardwareMap.getAll(LynxModule.class);
        for (int i = 0; i < foundHubs.size(); i = i + 1) {
            LynxModule hub = foundHubs.get(i);
            hubs.add(hub);
            // The "parent" hub is the one the Robot Controller talks to directly. On our robots
            // that is the Control Hub. The other one is the Expansion Hub.
            if (hub.isParent()) {
                hubNames.add("Control Hub");
            } else {
                hubNames.add("Expansion Hub");
            }
            voltagesNow.add(0.0);
            lowestVoltages.add(99.0);
            currentsNow.add(0.0);
            highestCurrents.add(0.0);
        }
    }

    // ==================================================================
    // ACTIONS
    // ==================================================================

    /*
     * Call this ONCE per loop(). It asks the hubs (only every SAMPLE_INTERVAL_SECONDS) and keeps
     * the numbers and the dip log up to date.
     */
    public void update() {
        if (hubs.isEmpty()) {
            return;
        }
        if (sampleTimer.seconds() < SAMPLE_INTERVAL_SECONDS) {
            return;
        }
        sampleTimer.reset();

        // The lowest voltage on ANY hub in this sample. A dip on either hub counts.
        double lowestThisSample = 99.0;
        String lowestHubName = "";

        for (int i = 0; i < hubs.size(); i = i + 1) {
            LynxModule hub = hubs.get(i);

            double volts = hub.getInputVoltage(VoltageUnit.VOLTS);
            if (volts > MIN_REAL_VOLTAGE_VOLTS) {
                voltagesNow.set(i, volts);
                if (volts < lowestVoltages.get(i)) {
                    lowestVoltages.set(i, volts);
                }
                if (volts < lowestThisSample) {
                    lowestThisSample = volts;
                    lowestHubName = hubNames.get(i);
                }
                if (volts < lowestEverVolts) {
                    lowestEverVolts = volts;
                }
            }

            if (READ_MOTOR_CURRENT) {
                double amps = hub.getCurrent(CurrentUnit.AMPS);
                currentsNow.set(i, amps);
                if (amps > highestCurrents.get(i)) {
                    highestCurrents.set(i, amps);
                }
            }
        }

        updateDipLog(lowestThisSample, lowestHubName);
    }

    /*
     * Forget the lowest and highest numbers and the dip count. Call it at START, so the numbers on
     * the screen are about THIS match and not about INIT or the last test.
     */
    public void resetStatistics() {
        for (int i = 0; i < hubs.size(); i = i + 1) {
            lowestVoltages.set(i, 99.0);
            highestCurrents.set(i, 0.0);
        }
        inDip = false;
        lowestInThisDip = 99.0;
        dipCount = 0;
        lowestEverVolts = 99.0;
        lastDipText = "none";
        sinceStartTimer.reset();
    }

    // ==================================================================
    // QUESTIONS
    // ==================================================================

    /*
     * The lowest voltage any hub showed in the newest sample. 0.0 if there is no reading yet.
     * The OpMode can put this in its own log lines (for example when something blinks).
     */
    public double getLowestVoltageNow() {
        double lowest = 99.0;
        boolean haveReading = false;
        for (int i = 0; i < voltagesNow.size(); i = i + 1) {
            double volts = voltagesNow.get(i);
            if (volts > MIN_REAL_VOLTAGE_VOLTS) {
                haveReading = true;
                if (volts < lowest) {
                    lowest = volts;
                }
            }
        }
        if (!haveReading) {
            return 0.0;
        }
        return lowest;
    }

    public int getDipCount() {
        return dipCount;
    }

    public boolean isInDip() {
        return inDip;
    }

    /*
     * One short line for the OpMode's log, like "battery now 12.4 V, lowest since START 11.1 V, 0 dips".
     */
    public String getSummaryText() {
        return String.format("battery now %.2f V, lowest since START %.2f V, %d dip(s) below %.1f V",
                getLowestVoltageNow(), lowestEverVolts, dipCount, DIP_THRESHOLD_VOLTS);
    }

    /*
     * Shows the battery and the dip log on the screen. Call it after update().
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- POWER (battery black box) ---");
        if (hubs.isEmpty()) {
            telemetry.addData("Hubs", "none found");
            return;
        }
        for (int i = 0; i < hubs.size(); i = i + 1) {
            String text = String.format("%.2f V now   lowest %.2f V", voltagesNow.get(i), lowestVoltages.get(i));
            if (READ_MOTOR_CURRENT) {
                text = text + String.format("   motors %.1f A now   highest %.1f A",
                        currentsNow.get(i), highestCurrents.get(i));
            }
            telemetry.addData(hubNames.get(i), text);
        }
        String nowText = "";
        if (inDip) {
            nowText = "   *** DIPPING RIGHT NOW ***";
        }
        telemetry.addData("Dips below " + DIP_THRESHOLD_VOLTS + " V", "%d%s", dipCount, nowText);
        telemetry.addData("Last dip", lastDipText);
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    /*
     * Starts, follows and ends the current dip, and writes each one to the Robot Controller log.
     * lowestThisSample is 99.0 when no hub gave a real reading, and then nothing changes.
     */
    private void updateDipLog(double lowestThisSample, String lowestHubName) {
        if (lowestThisSample > 90.0) {
            return;
        }

        if (!inDip) {
            if (lowestThisSample < DIP_THRESHOLD_VOLTS) {
                inDip = true;
                dipCount = dipCount + 1;
                dipTimer.reset();
                lowestInThisDip = lowestThisSample;
                BiobuzzLog.warning("BATTERY DIP started at %.1f s: %s read %.2f V (threshold %.1f V)",
                        sinceStartTimer.seconds(), lowestHubName, lowestThisSample, DIP_THRESHOLD_VOLTS);
            }
            return;
        }

        // We are in a dip. Track how low it goes, and end it once the voltage has recovered.
        if (lowestThisSample < lowestInThisDip) {
            lowestInThisDip = lowestThisSample;
        }
        if (lowestThisSample > DIP_THRESHOLD_VOLTS + DIP_END_MARGIN_VOLTS) {
            inDip = false;
            lastDipText = String.format("lowest %.2f V, lasted %.2f s, started %.1f s into the match",
                    lowestInThisDip, dipTimer.seconds(), sinceStartTimer.seconds() - dipTimer.seconds());
            BiobuzzLog.info("battery dip ended: %s", lastDipText);
        }
    }
}
