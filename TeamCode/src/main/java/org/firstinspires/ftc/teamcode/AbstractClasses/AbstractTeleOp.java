package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefishftcbridge.devices.*;
import com.roboctopi.cuttlefish.controller.*;

public abstract class AbstractTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

    }
    public abstract AbstractRobot instantiateRobot();
}
