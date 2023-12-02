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

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utilities.CASH_Drive_Library;
//Schultz Made this change
/**
 * This is the TeleOp Mode control class for 2021 robot.
 */

@TeleOp(name="CASH 2023-2024 TeleOp", group="Iterative Opmode")
//@Disabled
public class CASH2024Driver extends OpMode
{
    /*
     * Code to run ONCE when the driver hits INIT
     */
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loopTime = new ElapsedTime();
    private Robot2024 robot;
    public CASH_Drive_Library CASHDriveLibrary;

    //Teleop variables
    private double percentTurnPower = .5;

    private boolean dumping = false;

    private boolean raising = false;

    private double sweeperCmd = 0;

    private boolean AutoRaiseActiveLow = false;
    private boolean AutoRaiseActiveMid= false;
    private boolean AutoRaiseActiveHigh = false;
    private boolean doneGettingToPosition = false;

    private int auto_elevator_pos = 0;


    //Schultz Updates
    private boolean AutoElevatorActive = false;

    @Override

    public void init() {
        robot=new Robot2024(this);
      robot.initializeRobot();
       robot.initializeImplements2();
       CASHDriveLibrary=robot.CASHDriveLibrary;

        //Set the robot up to use encoders on all 4 wheels.  This is then velocity controlled which
        //can be a bit better than just open loop.
        CASHDriveLibrary.EnableEncoders();

        //This resets the angles to 0 degrees and will represent zero after robot is inicialized
        //This method can be used anytime you want to reset the angles to zero
        robot.robotIMU.resetAngle();

        telemetry.addData("Status", "Initialized");
        robot.setDesElevatorPosition_Teliop(robot.getElevatorPositition());
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }
    /*
     * Code to run ONCE when the driver hits PLAY.
     */
    @Override
    public void start() {
        runtime.reset();
        loopTime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        /*
        *Place your Teliop code here.
        * 1) Take inputs from joysticks
        * 2) Use the inputs from the joystick to move the robot forward/rev
         */
        double powerfactor;
        if (gamepad1.right_bumper == true ) {
            powerfactor = 0.5;
        }
        else {
            powerfactor = 1;

        }
        // INPUTS
        double drive_y = -gamepad1.left_stick_y * powerfactor;
        double drive_x = gamepad1.left_stick_x *  powerfactor;
        double turn_x = 0.75 * gamepad1.right_stick_x * powerfactor;
        double elevatorCommand = gamepad2.left_stick_y * -1;

        boolean reset_bucket = gamepad2.left_bumper;
        boolean dump_pixle = gamepad2.right_bumper;

        boolean arm = gamepad2.dpad_down;
        boolean fire = gamepad2.x;

        boolean low_elevator = gamepad2.a;
        boolean mid_elevator = gamepad2.b;
        boolean high_elevator = gamepad2.y;

        //sweeper control
        float in = gamepad1.right_trigger;
        float out = gamepad1.left_trigger;


        if (loopTime.milliseconds() > 20) {
            RobotLog.i(String.format("looptime: %.6f",loopTime.milliseconds()));
            loopTime.reset();

            if (in > .1) {
                sweeperCmd = 1.0;
            } else if (out > .1) {
                sweeperCmd = -0.5;
            } else {
                sweeperCmd = 0;
            }
            robot.sweeperCommand(sweeperCmd);

            //drone control
            if (arm && fire) {
                robot.launch_drone();
            } else if (fire) {
                robot.reset_launch();
            }

            if (reset_bucket) {
                robot.reset_pixle_bucket();
            }
            if (dump_pixle) {
                robot.dump_pixle();
            }

        /*if ( low_elevator == true && AutoRaiseActiveLow == false ) {
            auto_elevator_pos = 500;
            robot.setPosition(500);
            AutoRaiseActiveLow = true;
            AutoRaiseActiveMid = false;
            AutoRaiseActiveHigh = false;
            doneGettingToPosition = false;
            RobotLog.i(String.format("In Lower."));
        }

       else if ( mid_elevator == true && AutoRaiseActiveMid == false ) {
            auto_elevator_pos = 1000;
            robot.setPosition(1000);
            AutoRaiseActiveLow = false;
            AutoRaiseActiveMid = true;
            AutoRaiseActiveHigh = false;
            doneGettingToPosition = false;
            RobotLog.i(String.format("In Middle"));
        }

        else if ( high_elevator == true && AutoRaiseActiveHigh == false ) {
            auto_elevator_pos = 1900;
            robot.setPosition(1900);
            AutoRaiseActiveLow = false;
            AutoRaiseActiveMid = false;
            AutoRaiseActiveHigh = true;
            doneGettingToPosition = false;
            RobotLog.i(String.format("In High"));
        }


        if ((AutoRaiseActiveHigh || AutoRaiseActiveMid || AutoRaiseActiveLow) && !doneGettingToPosition )
        {RobotLog.i(String.format("In auto raise"));
            robot.raiseLowerElevator(1);
            if (robot.getElevatorPositition() > auto_elevator_pos){
                robot.raiseLowerElevator(0);
                doneGettingToPosition = true;
                RobotLog.i(String.format("In done auto raise"));

            }

        }
        else if (Math.abs(elevatorCommand) > 0.025){
            robot.raiseLowerElevator(elevatorCommand);
            AutoRaiseActiveLow = false;
            AutoRaiseActiveMid = false;
            AutoRaiseActiveHigh = false;
            doneGettingToPosition = false;
            RobotLog.i(String.format("In InNormal"));
        }
        else {
            robot.raiseLowerElevator(0);
            RobotLog.i(String.format("In HOLD"));
//            robot.elevatorHold(robot.getElevatorPositition(), loopTime.seconds(),1);
        }

        */

            //schultz update
            if (low_elevator == true) {
                robot.setDesElevatorPosition_Teliop(0);
                RobotLog.i(String.format("In Lower."));
//            AutoElevatorActive = true;
            } else if (mid_elevator == true) {
                robot.setDesElevatorPosition_Teliop(robot.ELEVATOR_MID_POSITION);
                RobotLog.i(String.format("In Middle"));
//            AutoElevatorActive = true;
            } else if (high_elevator == true) {
                robot.setDesElevatorPosition_Teliop(robot.ELEVATOR_HIGH_POSITION);
                RobotLog.i(String.format("In High"));
//            AutoElevatorActive = true;
            }

            if (Math.abs(elevatorCommand) > .025) {
                AutoElevatorActive = false;
                robot.raiseLowerElevator(elevatorCommand);
                robot.setDesElevatorPosition_Teliop(robot.getElevatorPositition());
            } else {
                robot.elevatorUpdate(loopTime.seconds());
            }

//        boolean BucketCommand = gamepad2.a;
//       boolean BucketCommand2 = gamepad2.y;
//      boolean SweepCommand = gamepad1.left_bumper;
//      boolean SweepCommand2 = gamepad1.left_bumper;
//       boolean stopSweeper = gamepad1.x;
//      boolean sweepCMD2 = gamepad1.left_bumper;
//        double SweeperCommand = 0;
//
//        if (!dumping && BucketCommand == true) {
//            robot.dump_bucket();
//           dumping = true;
//           raising = false;
//        }
//        if (!raising && BucketCommand2 == true) {
//            robot.raise_bucket();
//            dumping = false;
//            raising = true;
//        }
//       /// outtake true
//       /// intake  false
//        if (SweepCommand == true) {
//           SweeperCommand = 1;
//
//        } else if (SweepCommand2 == true) {
//            SweeperCommand = -1;
//        }else if (stopSweeper)
//        {
//            SweeperCommand = 0;
//        }


            robot.moveRobotteli(drive_y, drive_x, turn_x);


            telemetry.addData("elevator Commd", elevatorCommand);
            telemetry.addData("Status", "Running");
            telemetry.addData("#ofTicks", robot.getTicks());
            telemetry.update();
//        loopTime.reset();



        }
    }

}