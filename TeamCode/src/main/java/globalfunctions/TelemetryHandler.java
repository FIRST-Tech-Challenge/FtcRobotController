package globalfunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import autofunctions.Path;
import global.TerraBot;

public class TelemetryHandler {
    Telemetry telemetry;
    TerraBot bot;
    public void init(Telemetry telemetry, TerraBot bot){
        this.telemetry = telemetry;
        this.bot = bot;
    }

    public void addOdometry() {
//        telemetry.addData("center odometry", bot.odometry.cp);
//        telemetry.addData("left odometry", bot.odometry.lp);
//        telemetry.addData("right odometry", bot.odometry.rp);
        telemetry.addData("deltaRP", bot.odometry.deltaRP);
        telemetry.addData("deltaCP", bot.odometry.deltaCP);
        telemetry.addData("deltaLP", bot.odometry.deltaLP);
//        telemetry.addData("dl", "(%f , %f)", bot.odometry.dl.x, bot.odometry.dl.y);
//        telemetry.addData("dr", "(%f , %f)", bot.odometry.dr.x, bot.odometry.dr.y);
        telemetry.addData("deltaX", bot.odometry.deltaX);
        telemetry.addData("deltaY", bot.odometry.deltaY);

        telemetry.addData("x pos", bot.odometry.x);
        telemetry.addData("y pos", bot.odometry.y);
        telemetry.addData("heading", bot.odometry.h);

    }
    public void addAuton(Path path){
        telemetry.addData("y pow", path.yControl.getPower());
        telemetry.addData("x pow", path.xControl.getPower());
        telemetry.addData("h pow", path.hControl.getPower());
        telemetry.addData("y error", path.yControl.error);
        telemetry.addData("x error", path.xControl.error);
        telemetry.addData("h error", path.hControl.error);
        telemetry.addData("y derivative", path.yControl.derivative);
        telemetry.addData("x derivative", path.xControl.derivative);
        telemetry.addData("h derivative", path.hControl.derivative);
        telemetry.addData("y integral", path.yControl.integral);
        telemetry.addData("x integral", path.xControl.integral);
        telemetry.addData("h integral", path.hControl.integral);
        telemetry.addData("h", bot.odometry.h);
        telemetry.addData("x", bot.odometry.x);
        telemetry.addData("y", bot.odometry.y);
    }
    public void addAngularPosition() {
        telemetry.addData("average heading", bot.angularPosition.getHeading());
        telemetry.addData("average gyro angle", bot.angularPosition.getHeadingGY());
        telemetry.addData("left gyro", bot.angularPosition.getHeadingLeftGY());
        telemetry.addData("right gyro", bot.angularPosition.getHeadingRightGY());
        telemetry.addData("compass sensor", bot.angularPosition.getHeadingCS());
        telemetry.addData("cp raw", bot.angularPosition.compassSensor.getDirection());
    }
    public void addOuttake() {
        telemetry.addData("Right Outtake Position", bot.outr.getCurrentPosition());
        telemetry.addData("Left Outtake Position", bot.outl.getCurrentPosition());
        telemetry.addData("Right Outtake Angular Velocity", bot.autoAimer.outrController.currSpeed);
        telemetry.addData("Left Outtake Angular Velocity", bot.autoAimer.outlController.currSpeed);
//        telemetry.addData("Right Outtake Error", bot.autoAimer.outrController.pid.error);
//        telemetry.addData("Left Outtake Error", bot.autoAimer.outlController.pid.error);
//        telemetry.addData("Right Outtake Power", bot.autoAimer.outrController.power);
//        telemetry.addData("Left Outtake Power", bot.autoAimer.outlController.power);
//        telemetry.addData("Right Outtake Change Time", bot.autoAimer.outrController.changeTime);
//        telemetry.addData("Left Outtake Change Time", bot.autoAimer.outlController.changeTime);
        telemetry.addData("Right Outtake Derivative Power", bot.autoAimer.outrController.derivativeOfPower);
        telemetry.addData("Left Outtake Derivative Power", bot.autoAimer.outlController.derivativeOfPower);
        telemetry.addData("Right Outtake Target Speed", bot.autoAimer.outrController.targetSpeed);
        telemetry.addData("Left Outtake Target Speed", bot.autoAimer.outlController.targetSpeed);
    }
    public void addAutoAimer() {
        telemetry.addData("left distance", bot.localizer.getLeftDistance());
        telemetry.addData("back distance", bot.localizer.getBackDistance());
        telemetry.addData("Outl Target Power", bot.autoAimer.outlController.getMotorPower(bot.outl.getCurrentPosition()));
        telemetry.addData("Outr Target Power", bot.autoAimer.outrController.getMotorPower(bot.outr.getCurrentPosition()));
    }

    public Telemetry getTelemetry(){
        return telemetry;
    }
}
