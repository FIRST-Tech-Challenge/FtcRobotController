package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.revextensions2.ExpansionHubEx;

@TeleOp(name = "Voltage Monitor", group = "Test")
public class VoltageMonitor extends OpMode
{
    ExpansionHubEx expansionHub;

    @Override
    public void init()
    {
        /*
         * Before init() was called on this user code, REV Extensions 2
         * was notified via OpModeManagerNotifier.Notifications and
         * it automatically took care of initializing the new objects
         * in the hardwaremap for you. Historically, you would have
         * needed to call RevExtensions2.init()
         */
        hardwareMap.logDevices();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
    }

    @Override
    public void loop()
    {
        /*
         * ------------------------------------------------------------------------------------------------
         * Voltage monitors
         * ------------------------------------------------------------------------------------------------
         */

        String header =
                "**********************************\n" +
                        "VOLTAGE MONITOR          \n" +
                        "**********************************\n";
        telemetry.addLine(header);

        telemetry.addData("5v monitor", expansionHub.read5vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Voltage from the phone
        telemetry.addData("12v monitor", expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Battery voltage
        telemetry.update();
    }
}
