package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.FrenzyBotI;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

//@Disabled
@TeleOp(name = "Position To", group = "Robot15173")
public class PositionTest extends LinearOpMode {

    // Declare OpMode Members
    FrenzyBotI robot = new FrenzyBotI();

    // Timing related variables
    ElapsedTime runtime = new ElapsedTime();
    IBaseOdometry locator = null;

    @Override
    public void runOpMode() {
        try {
            try {
                robot.init(this, this.hardwareMap, telemetry);
                robot.initCalibData();
                locator =  VSlamOdometry.getInstance(this.hardwareMap, 20);
                locator.setInitPosition(50, 15, 180); // TODO: Remove this
                Thread odometryThread = new Thread(locator);
                odometryThread.start();

            } catch (Exception ex) {
                telemetry.addData("Init", ex.getMessage());
            }
            telemetry.update();


            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            Point target = new Point(65, 30);

            BotMoveProfile profile = BotMoveProfile.bestRoute(robot, (int)locator.getCurrentX(), (int)locator.getCurrentY(), target,
                    RobotDirection.Optimal, 0.5, MoveStrategy.Diag, -1, locator);


            robot.diagTo(profile);


            while (opModeIsActive()) {

                telemetry.addData("LeftTarget", profile.getLeftTarget());
                telemetry.addData("RightTarget", profile.getRightTarget());
                telemetry.addData("LeftTargetBack", profile.getLeftTargetBack());
                telemetry.addData("RightTargetBack", profile.getRightTargetBack());

                telemetry.addData("Left front", robot.getLeftOdometer());
                telemetry.addData("Right front", robot.getRightOdometer());
                telemetry.addData("Left Back", robot.getLeftBackOdometer());
                telemetry.addData("Right Back", robot.getRightBackOdometer());
                telemetry.update();
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
