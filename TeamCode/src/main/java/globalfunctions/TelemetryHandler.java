package globalfunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

import autofunctions.Path;
import global.TerraBot;

public class TelemetryHandler {
    Telemetry telemetry;
    TerraBot bot;
    public void init(Telemetry telemetry, TerraBot bot){
        this.telemetry = telemetry;
        this.bot = bot;
    }

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
    public void addAuton(Path path, int howMany){
        switch (howMany) {
            case 0:
                break;
            case 1:
                telemetry.addData("auto angle", bot.autoAimer.getRobotToGoalAngle(bot.odometry.getPos()));
                telemetry.addData("shotmode", bot.autoAimer.shotMode);
            case 2:
                telemetry.addData("y pow", path.yControl.getPower());
                telemetry.addData("x pow", path.xControl.getPower());
                telemetry.addData("h pow", path.hControl.getPower());
            case 3:
                telemetry.addData("y error", path.yControl.error);
                telemetry.addData("x error", path.xControl.error);
                telemetry.addData("h error", path.hControl.error);
            case 4:
                telemetry.addData("y derivative", path.yControl.derivative);
                telemetry.addData("x derivative", path.xControl.derivative);
                telemetry.addData("h derivative", path.hControl.derivative);
            case 5:
                telemetry.addData("y integral", path.yControl.integral);
                telemetry.addData("x integral", path.xControl.integral);
                telemetry.addData("h integral", path.hControl.integral);
        }
    }
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
                telemetry.addData("aimerPos", Arrays.toString(bot.aimerPos));
                telemetry.addData("autoAimerPos", Arrays.toString(bot.autoAimer.outtakePos));
            case 3:
                telemetry.addData("localizerChecksFailed", bot.localizer.checksFailed);
                telemetry.addData("gyroChecksFailed", bot.angularPosition.checksFailed);
            case 4:
                telemetry.addData("GameTime:", bot.gameTime.seconds());
        }

    }

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
