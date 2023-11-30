package org.firstinspires.ftc.teamcode.teamProp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeamPropOpMode extends LinearOpMode {
    TeamPropDetection teamPropDetection = new TeamPropDetection();
    @Override
    public void runOpMode() {

        teamPropDetection.Setup(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested())
        {

            teamPropDetection.GetPropLocation();
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }
}
