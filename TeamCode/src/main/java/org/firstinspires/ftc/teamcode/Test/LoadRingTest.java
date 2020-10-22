package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class LoadRingTest {

@TeleOp(name="LoadRing Test ", group="Teleop")
//@Disabled


    // Create instance of the lift subsystem. This opmode tests the lift motor and the limit switch

    //private ElapsedTime     runtime         =       new ElapsedTime();
Elevator elevator = new Elevator();
Intake intake = new Intake();
    @Override
    public void init() {
        // Call init methods for all implements needed in this opmode. Usually it will be all
        elevator.init(hardwareMap);
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
            elevator.ElevatorSpeedfast();
            telemetry.addData("Button X Pushed", "Complete ");
        }

        if (gamepad2.y) {
            elevator.ElevatorSpeedslow();
            telemetry.addData("Button Y Pushed", "Complete ");
        }

        if (gamepad2.b) {
            elevator.Elevatorbackup();

        }
        if (gamepad2.a) {
            elevator.Elevatoroff();
            //This turns off both the elevator and the intake
            intake.Intakeoff();
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }




}