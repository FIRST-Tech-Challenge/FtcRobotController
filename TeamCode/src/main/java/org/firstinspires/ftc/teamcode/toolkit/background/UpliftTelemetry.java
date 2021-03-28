package org.firstinspires.ftc.teamcode.toolkit.background;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.Background;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;
import org.firstinspires.ftc.teamcode.toolkit.misc.MathFunctions;

public class UpliftTelemetry extends Background {

    UpliftRobot robot;
    LinearOpMode opMode;
    Telemetry telem;

    public UpliftTelemetry(UpliftRobot robot) {
        super(robot);
        this.robot = robot;
        this.opMode = robot.opMode;
        this.telem = FtcDashboard.getInstance().getTelemetry();
//        this.telem = robot.opMode.telemetry;
    }

    @Override
    public void loop() {

//        if(opMode instanceof UpliftTele) {
//            displayTeleOpTelemetry(robot);
//        } else if(opMode instanceof UpliftAuto){
//            displayAutoTelemetry(robot);
//        }

    }

    public void displayTeleOpTelemetry(UpliftRobot robot) {
        if(robot.driveInitialized) {
            telem.addData("Current Pos:\t", "( " + MathFunctions.truncate(robot.worldX) + ", " + MathFunctions.truncate(robot.worldY) + " )");
            telem.addData("Current Angle:\t", robot.worldAngle);
            telem.addData("Left Encoder pos:\t", robot.odometry.getLeftTicks() / UpliftRobot.COUNTS_PER_INCH);
            telem.addData("Right Encoder pos:\t", robot.odometry.getRightTicks() / UpliftRobot.COUNTS_PER_INCH);
            telem.addData("Center Encoder pos:\t", robot.odometry.getCenterTicks() / UpliftRobot.COUNTS_PER_INCH);
            telem.addData("Slow Mode:\t", robot.slowMode);
            telem.addData("Shot Count\t", robot.shotCount);
            telem.addData("Transfer Pos:\t", robot.transferSub.transfer.getCurrentPosition());
            telem.addData("Transfer Mode:\t", robot.transfer.getMode());
            telem.addData("Current Mode:\t", robot.transfer.getCurrent(CurrentUnit.MILLIAMPS));
            telem.addData("Roller Pos:\t", robot.intakeToggle);
//            telem.addData("Distance Sensor:\t", robot.shooterSensor.getDistance(DistanceUnit.CM));
        }
        telem.addData("Shooting State\t",  robot.shootingState + "");
        telem.update();
    }

    public void displayAutoTelemetry(UpliftRobot robot) {
        if(robot.driveInitialized) {
            telem.addData("Current Pos:\t", "( " + MathFunctions.truncate(robot.worldX) + ", " + MathFunctions.truncate(robot.worldY) + " )");
            telem.addData("Current Angle:\t", robot.worldAngle);
            telem.addData("Left Encoder pos:\t", robot.odometry.getLeftTicks() / UpliftRobot.COUNTS_PER_INCH);
            telem.addData("Right Encoder pos:\t", robot.odometry.getRightTicks() / UpliftRobot.COUNTS_PER_INCH);
            telem.addData("Center Encoder pos:\t", robot.odometry.getCenterTicks() / UpliftRobot.COUNTS_PER_INCH);
        }
        if(robot.visionInitialized) {
            telem.addData("Ring Stack:\t", robot.ringDetector.ringCount);
        }
        telem.update();
    }

}
