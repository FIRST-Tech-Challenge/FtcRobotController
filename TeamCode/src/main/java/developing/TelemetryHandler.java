package developing;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryHandler {
    public Telemetry addOdometry(Telemetry telemetry, TestRobot bot) {
        telemetry.addData("center odometry", bot.getCenterOdo());
        telemetry.addData("left odometry", bot.getLeftOdo());
        telemetry.addData("right odometry", bot.getRightOdo());
        telemetry.addData("x pos", bot.odometry.getX());
        telemetry.addData("y pos", bot.odometry.getY());
        telemetry.addData("heading", bot.odometry.getHeading());
        return telemetry;
    }
    public Telemetry addAngularPosition(Telemetry telemetry, TestRobot bot) {
        telemetry.addData("average angle", bot.angularPosition.getHeading());
        telemetry.addData("average gyro angle", bot.angularPosition.getHeadingGY());
        telemetry.addData("left gyro", bot.angularPosition.getHeadingLeftGY());
        telemetry.addData("right gyro", bot.angularPosition.getHeadingRightGY());
        telemetry.addData("compass sensor", bot.angularPosition.getHeadingCS());
        return telemetry;
    }
    public Telemetry addOuttake(Telemetry telemetry, TestRobot bot) {
        telemetry.addData("Right Outtake Position", bot.outr.getCurrentPosition());
        telemetry.addData("Left Outtake Position", bot.outl.getCurrentPosition());
        telemetry.addData("Right Outtake Angular Velocity", bot.autoAimer.outrController.currSpeed);
        telemetry.addData("Left Outtake Angular Velocity", bot.autoAimer.outlController.currSpeed);
        telemetry.addData("Right Outtake Error", bot.autoAimer.outrController.currError);
        telemetry.addData("Left Outtake Error", bot.autoAimer.outlController.currError);
        telemetry.addData("Right Outtake Power", bot.autoAimer.outrController.power);
        telemetry.addData("Left Outtake Power", bot.autoAimer.outlController.power);
        telemetry.addData("Right Outtake Change Time", bot.autoAimer.outrController.changeTime);
        telemetry.addData("Left Outtake Change Time", bot.autoAimer.outlController.changeTime);
        telemetry.addData("Right Outtake Derivative Power", bot.autoAimer.outrController.derivativeOfPower);
        telemetry.addData("Left Outtake Derivative Power", bot.autoAimer.outlController.derivativeOfPower);
        telemetry.addData("Right Outtake Target Speed", bot.autoAimer.outrController.targetSpeed);
        telemetry.addData("Left Outtake Target Speed", bot.autoAimer.outlController.targetSpeed);
        return telemetry;
    }
    public Telemetry addAutoAimer(Telemetry telemetry, TestRobot bot) {
        telemetry.addData("left distance", bot.getLeftDistance());
        telemetry.addData("back distance", bot.getBackDistance());
        telemetry.addData("distance to center: ", bot.autoAimer.getDisFromCenter(bot.getLeftDistance(), bot.angularPosition.getHeadingGY()));
        telemetry.addData("Outl Target Power", bot.autoAimer.outlController.getMotorPower(bot.outl.getCurrentPosition()));
        telemetry.addData("Outr Target Power", bot.autoAimer.outrController.getMotorPower(bot.outr.getCurrentPosition()));
        return telemetry;
    }
}
