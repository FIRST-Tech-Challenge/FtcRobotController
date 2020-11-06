package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;



    @TeleOp(name="Shooter Only Test ", group="Teleop")
@Disabled
    public class Shooter_Test extends OpMode{


        // Create instance of the lift subsystem. This opmode tests the lift motor and the limit switch

        Shooter shooter            =    new Shooter();

        //private ElapsedTime     runtime         =       new ElapsedTime();

        @Override
        public void init() {
            // Call init methods for all implements needed in this opmode. Usually it will be all
            shooter.init(hardwareMap);
            telemetry.addData("Hardware is Initialized ", "Complete ");

        }

        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
        @Override
        public void init_loop() {
            // vision code to scan for objects would go here. Possibly encoder resets as well

        }

        /*
         * Code to run ONCE when the driver hits PLAY
         */
        @Override
        public void start() {
            // empty for this example
        }

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
        @Override
        public void loop() {

            //========================================
            // GAME PAD 2 Only for this test opMode
            //========================================

            // gripper assignment to X and Y buttons on implement gamepad
            // does not work 5/28. wires are in correct port too
            if (gamepad2.x) {
                shooter.shootMiddleGoal();
                telemetry.addData("Button X Pushed", "Complete ");
            }

            if (gamepad2.y) {
                shooter.shootHighGoal();
                telemetry.addData("Button Y Pushed", "Complete ");
            }
            if (gamepad2.a) {
                shooter.shooterOff();
                telemetry.addData("Button A Pushed", "Complete ");
            }

            if (gamepad2.b) {
                shooter.jamClear();

            }



        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
        }


    

}
