package org.firstinspires.ftc.teamcode;  //place where the code is located

//     o
//        o      ______/~/~/~/__           /((
//        o  // __            ====__    /_((
//        o  //  @))       ))))      ===/__((
//       ))           )))))))        __((
//       \\     \)     ))))    __===\ _((
//        \\_______________====      \_((
//        \((

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mainModules.Erection;
import org.firstinspires.ftc.teamcode.mainModules.MoveRobot;
import org.firstinspires.ftc.teamcode.mainModules.Presses;
import org.firstinspires.ftc.teamcode.mainModules.Gimbal;
import org.firstinspires.ftc.teamcode.mainModules.VisionManager;

@TeleOp(name = "Main code Estonia")
// allows to display the code in the driver station, comment out to remove
public class Estonia extends LinearOpMode { //file name is Main.java    extends the prebuilt LinearOpMode by rev to run
    /*
     * Import external classes
     * class name  name to be used in this class;
     *eg:
     * RunMotor runMotor;
     */
    MoveRobot moveRobot;
    Presses gamepad1_a;
    Presses gamepad1_b;
    Presses gamepad1_y;
    Presses gamepad2_dpad_up;
    Presses gamepad2_right_bumper;

    VisionManager visionManager;
    double[] positionData = {
            0, //x
            0, //y
            0, //rotation
    };

    Erection erection;

    Gimbal gimbal;

    @Override
    public void runOpMode() {
        /*
         * map objects
         * name to be used = new class()
         * eg:
         * runMotor = new RunMotor();
         *
         * if te external classes require initialisation do it here
         * eg:
         * runMotor.initRunMotor(hardwareMap);
         */

        moveRobot = new MoveRobot();
        moveRobot.initMoveRobot(hardwareMap, telemetry);

        gamepad1_a = new Presses();
        gamepad1_b = new Presses();
        gamepad1_y = new Presses();
        gamepad2_dpad_up = new Presses();
        gamepad2_right_bumper = new Presses();


        visionManager = new VisionManager();
        visionManager.initVision(hardwareMap, telemetry);

        gimbal = new Gimbal();
        gimbal.initGimbal(hardwareMap, telemetry);

        erection = new Erection();
        erection.initErection(hardwareMap, telemetry);

        telemetry.update();
        waitForStart(); //everything has been initialized, waiting for the start button

        while (opModeIsActive()) { // main loop

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            moveRobot.move(
                    drive, strafe, turn, // drive
                    gamepad1_a.toggle(gamepad1.a), // toggle field centric
                    gamepad1_b.toggle(gamepad1.b),
                    gamepad1_y.toggle(gamepad1.y)//toggle traction control
            );

            erection.raise(
                    gamepad2.right_stick_y, //raise back
                    gamepad2.a,
                    gamepad2.b,
                    gamepad2.y,
                    gamepad2.x
            );

            positionData = visionManager.returnPositionData(true);

            gimbal.moveGimbal(
                    gamepad2_dpad_up.toggle(gamepad2.dpad_up),
                    gamepad2.dpad_down,
                    gamepad2.left_stick_x,
                    gamepad2.left_stick_y,
                    positionData[3],
                    positionData[4],
                    positionData[5],
                    positionData[6]==1
                    );
            ;

            if (gamepad2.left_bumper) { gimbal.untuck(); }

            if (gamepad2.left_trigger > 0.5) { gimbal.tuck(); }

            gimbal.telemetryGimbal();

            //telemetry.addData("gamepad2.left_trigger", gamepad2.left_trigger);
            //telemetry.addData("field centric", gamepad1_a.returnToggleState());
            //telemetry.addData("traction control", gamepad1_b.returnToggleState());
            //telemetry.addData("PositionData", positionData[0]);
            telemetry.update();
            if (true){ //debugging
                while (true){
                    if (gamepad2_right_bumper.released(gamepad2.right_bumper)){
                        break;
                    } if (gamepad2.right_trigger >0.5) {
                        break;
                    }
                }
            }

        }
    }
}
