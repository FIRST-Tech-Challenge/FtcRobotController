package org.firstinspires.ftc.teamcode.OpModes.Util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


@Config //Allows ALL PUBLIC STATIC VARIABLES to be monitored on FTC Dash.
@TeleOp(name = "Example Teleop", group = "Example") //The name and group

public class TuningTeleop extends OpMode {

    //Pedro Pathing Follower

    private static DcMotorEx liftLeft;
    private static DcMotorEx liftRight;

    public static double P = 0, D = 0, F = 0.1, L = 0;
    PDFLController pdflController = new PDFLController(P, D, F, L);
    public static double power;

    public static int allowedError = 1;
    public static UniConstants.EncoderReadingFrom readingFrom = UniConstants.EncoderReadingFrom.BOTH ;

    private static double currentPosition = 0;

    public static int liftTarget = 0;
    @Override
    public void init() {
        //ONE TIME INIT CALL
        //This is where you will call all of the constructors for your different subsystems.
        //Eventually, once I have them setup, you will see examples like intake, outtake, etc.
        //Telemetry is also reinitialized here as to allow you to view it on FTC Dashboard (http://192.168.43.1:8080/dash)
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        liftLeft = hardwareMap.get(DcMotorEx.class, UniConstants.SLIDE_MOTOR_LEFT);
        liftRight = hardwareMap.get(DcMotorEx.class, UniConstants.SLIDE_MOTOR_RIGHT);
        liftLeft.setDirection(UniConstants.SLIDE_MOTOR_LEFT_DIRECTION);
        liftRight.setDirection(UniConstants.SLIDE_MOTOR_RIGHT_DIRECTION);

    }




    @Override
    public void init_loop() {
        telemetry.addData("P ", P);
        telemetry.addData("D ", D);
        telemetry.addData("F ", F);
        telemetry.addData("L ", L);
        telemetry.addData("Enabled ", FtcDashboard.getInstance().isEnabled());
        telemetry.update();
    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {


        currentPosition = getCurrentPosition();
        pdflController.setPDFL(P, D, F, L);
        pdflController.setTarget(liftTarget);
        pdflController.update(currentPosition);
        power = pdflController.runPDFL(allowedError);
        liftLeft.setPower(-power);
        liftRight.setPower(power);

        telemetry.addData("Lift Power Supplied ", power);
        telemetry.addData("Lift Left Encoder ", liftLeft.getCurrentPosition());
        telemetry.addData("Lift Right Encoder ", liftRight.getCurrentPosition());
        telemetry.addData("Lift Target ", liftTarget);
        telemetry.addData("Lift Reading From ", readingFrom);
        telemetry.addData("Lift Current Read Position ", getCurrentPosition());
        telemetry.update();




    }

    public static int getCurrentPosition(){
        return readingFrom == UniConstants.EncoderReadingFrom.LEFT ? liftLeft.getCurrentPosition()   :
                (int) (readingFrom == UniConstants.EncoderReadingFrom.RIGHT ? liftRight.getCurrentPosition() :
                        (double) (liftLeft.getCurrentPosition() + liftRight.getCurrentPosition()) / 2);
    }
}
