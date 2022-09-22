package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes2020;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ultimategoal2020.DriveWheel;
import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020.EbotsManip2020;

import java.util.ArrayList;

import static java.lang.String.format;

import android.util.Log;

@TeleOp(name="2021 Teleop", group="Dev")
@Disabled
public class EbotsTeleOp2020 extends LinearOpMode {

    // Declare OpMode members.
    private StopWatch stopWatch;
    private EbotsRobot2020 robot;
    private int loopCount = 0;
    private boolean debugOn = true;
    private String logTag = "EBOTS";


    @Override
    public void runOpMode() throws InterruptedException {
        //Configure FtcDashboard telemetry
//        dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = dashboard.getTelemetry();
//        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        // This constructor initializes driveWheels and manip devices
        if (debugOn){
            Log.d(logTag, "EbotsTeleOp2021::runOpMode...");
        }
        robot = new EbotsRobot2020(hardwareMap);
        robot.setAlliance(Alliance.RED);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        waitForStart();

        // Once the start button is pressed, reset some variables
        stopWatch = new StopWatch();
        telemetry.clearAll();

        // enter the control loop
        while(opModeIsActive()){
            loopCount++;
            robot.setDriveCommand(gamepad1);        //Sets command and runs calculateDrivePowers
            //robot.calculateDrivePowers();
            robot.drive();
            // robot.handleManipInput(gamepad2);
            handleManipInput(gamepad2);
            updateTelemetry();
        }
    }

    private void handleManipInput(Gamepad gamepad){
        // Get an array of all the manip devices
        ArrayList<EbotsManip2020> ebotsManips = robot.getEbotsManips();

        // Send each manip device the gamepad inputs to process
        for(EbotsManip2020 m: ebotsManips){
            m.handleGamepadInput(gamepad2);
        }
    }

    public void updateTelemetry(){
        if (debugOn){
            Log.d(logTag, "EbotsTeleOp2021::updateTelemetry");
        }
        // Show the elapsed game time and wheel power.
        String f = "%.2f";
        telemetry.addData("Status", "Run Time: " + stopWatch.getElapsedTimeSeconds());
        telemetry.addData("Crane Power", String.format(f, robot.getCrane().getPower()));
        telemetry.addData("Shooter Power/Velocity:  ",
                format(f, robot.getLauncher().getPower()) + " / "
                        + format(f, robot.getLauncher().getVelocity()));
        telemetry.addData("Crane Position", robot.getCrane().getCurrentPosition());

        telemetry.addData("ringFeeder Position", robot.getRingFeeder().getPosition());
        telemetry.addData("ringFeeder cycle timer:", robot.getRingFeederCycleTimer().getElapsedTimeMillis());


        for (DriveWheel dw: robot.getDriveWheels()){
            int encoderClicks = dw.getEncoderClicks();
            telemetry.addData(dw.getWheelPosition() + " encoder Clicks: ", encoderClicks);
        }

        telemetry.addLine(stopWatch.toString(loopCount));
        telemetry.update();
    }
}
