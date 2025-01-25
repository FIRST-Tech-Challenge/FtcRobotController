package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

import java.util.Locale;

@Autonomous(name = "Auto: Right Side 4 Specimens", group = "Techtonics")
public class TT_AutonomousRightSide_4_Specimens extends TT_AutonomousRightSide {
    @Override
    public void runOpMode() {
        Four_Specimans = true;
        runGeneralOpMode();
    }
}
