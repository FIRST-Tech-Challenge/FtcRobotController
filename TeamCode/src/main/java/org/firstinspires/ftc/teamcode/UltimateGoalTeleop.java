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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.configs.lessButtonsConfig;
import org.firstinspires.ftc.teamcode.configs.teleConfigEx;
import org.firstinspires.ftc.teamcode.configs.teleConfigRohit;
import org.firstinspires.ftc.teamcode.configs.teleConfigRohit2;
import org.firstinspires.ftc.teamcode.configs.teleConfigSamih;
import org.firstinspires.ftc.teamcode.configs.teleConfigTESTING_R;
import org.firstinspires.ftc.teamcode.configs.teleConfigTESTING_S;
import org.firstinspires.ftc.teamcode.configs.teleOpInterface;

import java.util.ArrayList;


/**
 * This file provides basic Telop driving for a RoverRuckus robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common RoverRuckus hardware class to define the devices on the robot.
 * All device access is managed through the HardwareRoverRuckus class.
 *
 */

@TeleOp(name="Ultimate Goal TeleOp", group="UltimateGoal")
public class UltimateGoalTeleop extends OpMode{


    // declaring variables
    MecanumDriveTrain vroom;
    Drivetrain drivetrain;

    /* Declare OpMode members. */
    HardwareMapV2 robot = new HardwareMapV2(true); // use the class created to define a RoverRuckus's hardware

    teleOpInterface t = new teleConfigEx(robot);
    ArrayList<Class<? extends teleOpInterface>> configs = new ArrayList<>();
    int index = 0;
    double duTimer, ddTimer;
    boolean switching = false;
    static final double     P_TURN_COEFF            = 0.03;

    double startingAngle, turnTimer;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        drivetrain = new Drivetrain(robot);
        robot.init(hardwareMap);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Set to REVERSE if using AndyMark motors
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);// Set to FORWARD if using AndyMark motors
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // initializing the variables


        vroom = new MecanumDriveTrain(robot, gamepad1,telemetry);

        configs.add(teleConfigEx.class);
        configs.add(teleConfigRohit.class);
        configs.add(teleConfigSamih.class);
        configs.add(teleConfigRohit2.class);
        configs.add(lessButtonsConfig.class);
        configs.add(teleConfigTESTING_R.class);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Haddi", "Haddi");
        telemetry.update();

        startingAngle = drivetrain.getAverageGyro();

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //Mecanum Drivetrain function to set powers
        vroom.loop();

        if (gamepad2.start){
            switching = true;
        }

        if (switching) {
            if (gamepad2.dpad_down && index!=(configs.size()-1) && System.currentTimeMillis()-ddTimer>=500){index++; ddTimer = System.currentTimeMillis();}
            if (gamepad2.dpad_up && index!=0 && System.currentTimeMillis()-duTimer>=500){index--; duTimer = System.currentTimeMillis();}
            for (Class<? extends teleOpInterface> t : configs){
                telemetry.addData(t.getName(), (index==configs.indexOf(t)) ? " <" : "");
            }
            telemetry.update();
            if (gamepad2.x) {
                if (configs.get(index).getName().equals(teleConfigEx.class.getName())) {
                    t = new teleConfigEx(robot);
                } else if (configs.get(index).getName().equals(teleConfigRohit.class.getName())) {
                    t = new teleConfigRohit(robot);
//              }else if (configs.get(index).getName().equals(teleConfigSamih.class.getName())){
//                  t = new teleConfigSamih(robot);
                } else if (configs.get(index).getName().equals(teleConfigRohit2.class.getName())) {
                    t = new teleConfigRohit2(robot);
                } else if (configs.get(index).getName().equals(lessButtonsConfig.class.getName())) {
                    t = new lessButtonsConfig(robot);
                } else if (configs.get(index).getName().equals(teleConfigTESTING_R.class.getName())) {
                    t = new teleConfigTESTING_R(robot);
                } else if (configs.get(index).getName().equals(teleConfigTESTING_S.class.getName())) {
                    t = new teleConfigTESTING_S(robot);
                }
                switching = false;
            }
        } else {
            t.a(gamepad2.a);
            t.b(gamepad2.b);
            t.x(gamepad2.x);
            t.y(gamepad2.y);

            t.dd(gamepad2.dpad_down);
            t.dp(gamepad2.dpad_up);
            t.dr(gamepad2.dpad_right);
            t.dl(gamepad2.dpad_left);

            t.rb(gamepad2.right_bumper);
            t.lb(gamepad2.left_bumper);

            t.lt(gamepad2.left_trigger);
            t.rt(gamepad2.right_trigger);

            t.rjoy(gamepad2.right_stick_x, gamepad2.right_stick_y);
            t.ljoy(gamepad2.left_stick_x, gamepad2.left_stick_y);
            t.rjoyb(gamepad2.right_stick_button);
            t.ljoyb(gamepad2.left_stick_button);

            for (String caption : t.telemetryDM.keySet()){
                telemetry.addData(caption, t.telemetryDM.get(caption));
            }
            telemetry.addData("Configuration: ", t.getName());
            telemetry.addData("FL", robot.frontLeft.getCurrentPosition());
            telemetry.addData("FR", robot.frontRight.getCurrentPosition());
            telemetry.addData("BL", robot.backLeft.getCurrentPosition());
            telemetry.addData("BR", robot.backRight.getCurrentPosition());
            telemetry.addData("Gpad 1 a?", gamepad1.a);
            telemetry.update();
            t.updateTelemetryDM();

            t.loop();

            if (gamepad1.a){
                turnTimer = System.currentTimeMillis() + 1500;
                gyroTurn(0.5, startingAngle-15);
                telemetry.addData("ok", "yes");
                telemetry.update();
            }

        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void changeConfig() throws InterruptedException {
        while (!gamepad2.x) {
            if (gamepad2.dpad_down && index!=(configs.size()-1) && System.currentTimeMillis()-ddTimer>=500){index++; ddTimer = System.currentTimeMillis();}
            if (gamepad2.dpad_up && index!=0 && System.currentTimeMillis()-duTimer>=500){index--; duTimer = System.currentTimeMillis();}
            for (Class<? extends teleOpInterface> t : configs){
                telemetry.addData(t.getName(), (index==configs.indexOf(t)) ? " <" : "");
            }
            telemetry.update();
        }
        if (configs.get(index).getName().equals(teleConfigEx.class.getName())){
            t = new teleConfigEx(robot);
        }else if (configs.get(index).getName().equals(teleConfigRohit.class.getName())){
            t = new teleConfigRohit(robot);
        }else if (configs.get(index).getName().equals(teleConfigSamih.class.getName())){
            t = new teleConfigSamih(robot);
        }else if (configs.get(index).getName().equals(teleConfigRohit2.class.getName())){
            t = new teleConfigRohit2(robot);
        }else if (configs.get(index).getName().equals(lessButtonsConfig.class.getName())) {
            t = new lessButtonsConfig(robot);
        }
    }
    // turn to an angle using gyro
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (!gamepad1.x && !drivetrain.onHeading(speed, angle, P_TURN_COEFF) && System.currentTimeMillis() < turnTimer) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("current_heading", drivetrain.getAverageGyro());
            telemetry.update();
        }
    }

}