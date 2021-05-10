package globalfunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

import autofunctions.Path;
import global.TerraBot;

public class TelemetryHandler {
    //Telemetry and bot
    Telemetry telemetry;
    TerraBot bot;
    //Init with variables
    public void init(Telemetry telemetry, TerraBot bot){
        this.telemetry = telemetry;
        this.bot = bot;
    }
    //Adds odometry
    public void addOdometry(int howMany) {
        switch (howMany) {
            case 0:
                break;
            case 1:
                telemetry.addData("center odometry", bot.odometry.cp);
                telemetry.addData("left odometry", bot.odometry.lp);
                telemetry.addData("right odometry", bot.odometry.rp);
            case 2:
                telemetry.addData("x pos", bot.odometry.x);
                telemetry.addData("y pos", bot.odometry.y);
                telemetry.addData("heading", bot.odometry.h);
            case 3:
                telemetry.addData("deltaX", bot.odometry.deltaX);
                telemetry.addData("deltaY", bot.odometry.deltaY);
            case 4:
                telemetry.addData("deltaRP", bot.odometry.deltaRP);
                telemetry.addData("deltaCP", bot.odometry.deltaCP);
                telemetry.addData("deltaLP", bot.odometry.deltaLP);
        }
    }
    //Adds auton
    public void addAuton(Path path, int howMany){
        switch (howMany) {
            case 0:
                break;
            case 1:
                telemetry.addData("auto angle", bot.autoAimer.getRobotToGoalAngle(bot.odometry.getPos()));
                telemetry.addData("shotmode", bot.autoAimer.shotMode);
            case 2:
                telemetry.addData("y pow", path.waypointController.yControl.getPower());
                telemetry.addData("x pow", path.waypointController.xControl.getPower());
                telemetry.addData("h pow", path.waypointController.hControl.getPower());
            case 3:
                telemetry.addData("y error", path.waypointController.yControl.error);
                telemetry.addData("x error", path.waypointController.xControl.error);
                telemetry.addData("h error", path.waypointController.hControl.error);
            case 4:
                telemetry.addData("y derivative", path.waypointController.yControl.derivative);
                telemetry.addData("x derivative", path.waypointController.xControl.derivative);
                telemetry.addData("h derivative", path.waypointController.hControl.derivative);
            case 5:
                telemetry.addData("y integral", path.waypointController.yControl.integral);
                telemetry.addData("x integral", path.waypointController.xControl.integral);
                telemetry.addData("h integral", path.waypointController.hControl.integral);
            case 6:
                telemetry.addData("x done", path.waypointController.xControl.isDone());
                telemetry.addData("y done", path.waypointController.yControl.isDone());
                telemetry.addData("h done", path.waypointController.hControl.isDone());
            case 7:
                telemetry.addData("x acc", path.waypointController.xControl.acc);
                telemetry.addData("y acc", path.waypointController.yControl.acc);
                telemetry.addData("h acc", path.waypointController.hControl.acc);
        }
    }
    //Adds angular position
    public void addAngularPosition(int howMany) {
        switch (howMany) {
            case 0:
                break;
            case 1:
                telemetry.addData("average gyro angle", bot.angularPosition.getHeadingGY());
            case 2:
                telemetry.addData("left gyro", bot.angularPosition.getHeadingLeftGY());
                telemetry.addData("right gyro", bot.angularPosition.getHeadingRightGY());
            case 3:
                telemetry.addData("compass sensor", bot.angularPosition.getHeadingCS());
        }
    }
    //Adds outtake
    public void addOuttake(int howMany) {
        switch (howMany) {
            case 0:
                break;
            case 1:
                telemetry.addData("Right Outtake Position", bot.outr.getCurrentPosition());
                telemetry.addData("Left Outtake Position", bot.outl.getCurrentPosition());
            case 2:
                telemetry.addData("Right Outtake Angular Velocity", bot.getRightAngVel());
                telemetry.addData("Left Outtake Angular Velocity", bot.getLeftAngVel());
        }
    }
    //Adds autoaimer
    public void addAutoAimer(int howMany) {
        switch (howMany) {
            case 0:
                break;
            case 1:
                telemetry.addData("x", bot.localizer.getPos()[0]);
                telemetry.addData("y", bot.localizer.getPos()[1]);
            case 2:
                telemetry.addData("back distance", bot.localizer.getBackDistance());
                telemetry.addData("left distance", bot.localizer.getLeftDistance());
                break;
            case 3:
                telemetry.addData("General Target Speed", bot.autoAimer.targetSpeed);
                telemetry.addData("Outl Target Speed", bot.autoAimer.getOutlTargetVel() * Constants.pi2/Constants.GOBUILDA1_Ticks); // 166
                telemetry.addData("Outr Target Speed", bot.autoAimer.getOutrTargetVel() * Constants.pi2/Constants.GOBUILDA1_Ticks); // 179
                telemetry.addData("Angle to goal", bot.getRobotToGoalAngle());
        }

    }
    //Adds other
    public void addOther(int howMany) {
        switch (howMany) {
            case 0:
                break;
            case 1:
                telemetry.addData("Powershot Mode", bot.powershotMode);
                telemetry.addData("Can Move", bot.isMovementAvailable);
            case 2:
                telemetry.addData("hasReached", bot.autoAimer.hasReached);
                telemetry.addData("Dpos", Arrays.toString(bot.localizer.getPos()));
                telemetry.addData("odometryPos", Arrays.toString(bot.odometry.getPos()));
                telemetry.addData("autoAimerPos", Arrays.toString(bot.autoAimer.outtakePos));
            case 3:
                telemetry.addData("localizerChecksFailed", bot.localizer.checksFailed);
                telemetry.addData("gyroChecksFailed", bot.angularPosition.checksFailed);
            case 4:
                telemetry.addData("GameTime:", bot.gameTime.seconds());
        }

    }
    //Adds everything for teleop
    public void addTele(int odometry, int angpos, int outtake, int autoaimer, int other){
        addOdometry(odometry);
        addAngularPosition(angpos);
        addOuttake(outtake);
        addAutoAimer(autoaimer);
        addOther(other);
    }



    public Telemetry getTelemetry(){
        return telemetry;
    }
}
