package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Frenzy Blue", group = "Robot15173")
public class FrenzyModeRed extends FrenzyModeBase {

    protected String getSide(){
        return AutoRoute.NAME_BLUE;
    }


}
