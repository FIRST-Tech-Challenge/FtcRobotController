package org.firstinspires.ftc.teamcode.teamProp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeamPropOpMode extends LinearOpMode {
    TeamProp teamProp = new TeamProp();
    @Override
    public void runOpMode() {

        teamProp.Setup(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested())
        {

            teamProp.GetPropLocation();
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }
}
