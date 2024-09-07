package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AbstractClasses.*;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.PeppyFeetFiend;

import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefish.utils.Pose;
import com.roboctopi.cuttlefishftcbridge.devices.*;
import com.roboctopi.cuttlefish.controller.*;
import com.roboctopi.cuttlefishftcbridge.opmodeTypes.GamepadOpMode;

public abstract class AbstractTeleOp extends AbstractOpMode {

    PeppyFeetFiend robot = new PeppyFeetFiend(this);

    public void onInit(){
        super.onInit();
    }

    public abstract AbstractRobot instantiateRobot();
}
