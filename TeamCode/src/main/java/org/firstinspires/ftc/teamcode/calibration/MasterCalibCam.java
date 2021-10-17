package org.firstinspires.ftc.teamcode.calibration;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;
import org.firstinspires.ftc.teamcode.bots.OdoBot;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.bots.RobotMovementStats;
import org.firstinspires.ftc.teamcode.bots.RobotVeer;
import org.firstinspires.ftc.teamcode.bots.YellowBot;
import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.skills.Geometry;

import java.util.concurrent.TimeUnit;

//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;


@TeleOp(name="MasterCalib Cam", group="Robot15173")
public class MasterCalibCam extends MasterCalib {

    protected void initBot(){
        this.bot = new FrenzyBot();
    }

    protected IBaseOdometry initLocator(){
        RobotCoordinatePosition locator = new RobotCoordinatePosition(bot, new Point(startX, startY), lastOrientation, RobotCoordinatePosition.THREAD_INTERVAL);
        locator.reverseHorEncoder();
        Thread positionThread = new Thread(locator);
        positionThread.start();
        return locator;
    }

}
