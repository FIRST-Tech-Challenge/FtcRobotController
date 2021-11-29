package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

//@Disabled
@TeleOp(name = "Color Test", group = "Robot15173")
public class PositionTest extends LinearOpMode {

    // Declare OpMode Members
    FrenzyBot robot = new FrenzyBot();

    // Timing related variables
    ElapsedTime runtime = new ElapsedTime();
    IBaseOdometry locator = null;

    @Override
    public void runOpMode() {
        try {
            try {
                robot.init(this, this.hardwareMap, telemetry);
                robot.initCalibData();


            } catch (Exception ex) {
                telemetry.addData("Init", ex.getMessage());
            }
            telemetry.update();


            // Wait for the game to start (driver presses PLAY)
            waitForStart();



            while (opModeIsActive()) {

              robot.detectColor(telemetry, 0) ;
            }
        } catch (Exception ex) {
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
            sleep(1000);
        }
        finally {
            if (locator != null){
                locator.stop();
            }
        }
    }


}
