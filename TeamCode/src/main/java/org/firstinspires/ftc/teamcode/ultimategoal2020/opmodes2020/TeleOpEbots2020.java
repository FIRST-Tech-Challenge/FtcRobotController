/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes2020;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ultimategoal2020.DriveWheel;
import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

import static java.lang.String.format;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop Ebots", group="Dev")
@Disabled
public class TeleOpEbots2020 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private StopWatch stopWatch;
    private EbotsRobot2020 robot;
    private int loopCount = 0;

//    FtcDashboard dashboard;
//    Telemetry dashboardTelemetry;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //Configure FtcDashboard telemetry
//        dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = dashboard.getTelemetry();
//        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        robot = new EbotsRobot2020();
        //robot.initDriveMotors(hardwareMap);
        robot.initializeStandardDriveWheels(hardwareMap);
//        robot.initializeEncoderTrackers(EncoderSetup.TWO_WHEELS, true);
//        robot.initializeExpansionHubsForBulkRead(hardwareMap);

        robot.initializeManipMotors(this.hardwareMap);
//        PIDFCoefficients cranePIDF = new PIDFCoefficients(2.5,0,0,0);
//        robot.getCrane().setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, cranePIDF);

        robot.setAlliance(Alliance.RED);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
        stopWatch = new StopWatch();
        telemetry.clearAll();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        loopCount++;
        robot.setDriveCommand(gamepad1);        //Sets command and runs calculateDrivePowers
        //robot.calculateDrivePowers();
        robot.drive();
        robot.handleManipInput(gamepad2);

        // Show the elapsed game time and wheel power.
        String f = "%.2f";
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Crane Power", String.format(f, robot.getCrane().getPower()));
        //DriveWheel driveWheel = robot.getDriveWheel(DriveWheel.WheelPosition.FRONT_LEFT);
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

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }

}
