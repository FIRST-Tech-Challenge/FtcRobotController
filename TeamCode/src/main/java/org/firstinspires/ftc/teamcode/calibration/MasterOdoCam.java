package org.firstinspires.ftc.teamcode.calibration;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.autonomous.AutoStep;
import org.firstinspires.ftc.teamcode.bots.BotAction;
import org.firstinspires.ftc.teamcode.bots.BotActionObj;
import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.bots.UltimateBot;
import org.firstinspires.ftc.teamcode.odometry.OdoBase;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;
import org.firstinspires.ftc.teamcode.skills.Led;

import java.io.File;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.autonomous.AutoRoute.NAME_BLUE;
import static org.firstinspires.ftc.teamcode.autonomous.AutoRoute.NAME_RED;
import static org.firstinspires.ftc.teamcode.autonomous.AutoStep.NO_ACTION;

@TeleOp(name="Master Odo Cam", group="Robot15173")
public class MasterOdoCam extends MasterOdo {
    @Override
    protected void initBot() {
        this.bot = new FrenzyBot();
    }

    @Override
    protected void initLocator() {
        this.locator = VSlamOdometry.getInstance(hardwareMap);
        this.locator.init(new Point(startX, startY), initHead);
        startLocator(locator);
    }
}
