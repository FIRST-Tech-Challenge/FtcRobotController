package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeamDetection {
    public boolean redTeam;
    public boolean blueTeam;

    /**
     * Returns what team you are on from the switch.
     * requires a switch configured as "blueSwitch"
     * Blue is true, and Red is false
     *
     * @param hardwareMap OpMode hardwareMap
     *
     */
    public TeamDetection(HardwareMap hardwareMap){

        //find value of switch
        DigitalChannel blueSwitch = hardwareMap.get(DigitalChannel.class, "blueSwitch");
        blueSwitch.setMode(DigitalChannel.Mode.INPUT);

        //set two values according to the switch
        blueTeam = blueSwitch.getState();
        redTeam = !blueTeam;
    }

    /**
     * Broadcasts variables:
     * blueTeam (true/false)
     * redTeam (true/false)
     *
     * @param telemetry OpMode telemetry
     */
    public void showTeam(Telemetry telemetry) {

        //let the driver know what it detected
        if(blueTeam) {
            telemetry.addData("Team:Blue", true);
            telemetry.addData("Team:Red", false);
        }

        if(redTeam) {
            telemetry.addData("Team:Blue", false);
            telemetry.addData("Team:Red", true);
        }
    }
}
//boom! now we know what team we're on!