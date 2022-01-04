package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


//@Disabled
//@Autonomous(name="Move: Test", group="Test")
public class MoveTest extends OpMode{

    private int stateMachineFlow;
    MecanumDrive robot = new MecanumDrive();

    double time;
    private ElapsedTime     runtime = new ElapsedTime();
    /***********************************
     *
     * This program tests each direction the robot can move
     *
     ***********************************/
    @Override
    public void init() {
        msStuckDetectInit = 11500;
        msStuckDetectLoop = 25000;

        robot.init(hardwareMap);
        robot.initIMU(hardwareMap);

        stateMachineFlow = 0;
    }

    @Override
    public void loop() {
        switch (stateMachineFlow) {
            case 0:
                runtime.reset();
                time = getRuntime();
                stateMachineFlow++;
                break;
            case 1:
                //move forward
//                robot.frontLinearDrive(.5,-25);
//                telemetry.addData("Rf",robot.getRFencoder());
//                telemetry.addData("Lf",robot.getLFencoder());
//                telemetry.addData("Rb",robot.getRBencoder());
//                telemetry.addData("Lb",robot.getLBencoder());
//                telemetry.update();
//                time = runtime.time();
                stateMachineFlow++;
                break;
            case 2:
                //wait 3 seconds
//                while (3 > runtime.time() - time) {
//
//                }
                stateMachineFlow++;
                break;
            case 3:
                //move backwards
//                robot.frontLinearDrive(.5,25);
//                telemetry.addData("Rf",robot.getRFencoder());
//                telemetry.addData("Lf",robot.getLFencoder());
//                telemetry.addData("Rb",robot.getRBencoder());
//                telemetry.addData("Lb",robot.getLBencoder());
//                telemetry.update();
//                time = runtime.time();
                stateMachineFlow++;
                break;
            case 4:
                //wait 3 seconds
//                while (3 > runtime.time() - time) {
//
//                }
                stateMachineFlow++;
                break;
            case 5:
                //move forward
                robot.linearDrive(.5,-25);
                telemetry.addData("Rf",robot.getRFencoder());
                telemetry.addData("Lf",robot.getLFencoder());
                telemetry.addData("Rb",robot.getRBencoder());
                telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 6:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 7:
                //move backwards
                robot.linearDrive(.5,25);
                telemetry.addData("Rf",robot.getRFencoder());
                telemetry.addData("Lf",robot.getLFencoder());
                telemetry.addData("Rb",robot.getRBencoder());
                telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 8:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 9:
                //move right
                robot.sideDrive(.8,-40);
                //telemetry.addData("Rf",robot.getRFencoder());
                telemetry.addData("Lf",robot.getLFencoder());
                //telemetry.addData("Rb",robot.getRBencoder());
                //telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 10:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 11:
                //move left
                robot.sideDrive(.8,40);
                //telemetry.addData("Rf",robot.getRFencoder());
                telemetry.addData("Lf",robot.getLFencoder());
                //telemetry.addData("Rb",robot.getRBencoder());
                //telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 12:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 13:
                //up and to the right
                robot.diagonalDrive(.5,-20, DiagonalDirection.RIGHT);
                //telemetry.addData("Rf",robot.getRFencoder());
                telemetry.addData("Lf",robot.getLFencoder());
                telemetry.addData("Rb",robot.getRBencoder());
                //telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 14:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 15:
                //down and to the left
                robot.diagonalDrive(.5,20, DiagonalDirection.LEFT);
                //telemetry.addData("Rf",robot.getRFencoder());
                telemetry.addData("Lf",robot.getLFencoder());
                telemetry.addData("Rb",robot.getRBencoder());
                //telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 16:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 17:
                //up and to the left
                robot.diagonalDrive(.5, -20, DiagonalDirection.LEFT);
                telemetry.addData("Rf",robot.getRFencoder());
                //telemetry.addData("Lf",robot.getLFencoder());
                //telemetry.addData("Rb",robot.getRBencoder());
                telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 18:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 19:
                //down and to the right
                robot.diagonalDrive(.5, 20, DiagonalDirection.RIGHT);
                telemetry.addData("Rf",robot.getRFencoder());
                //telemetry.addData("Lf",robot.getLFencoder());
                //telemetry.addData("Rb",robot.getRBencoder());
                telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 20:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 21:
                //turn 90 degrees to the right
                robot.gStatTurn(.5,-90);
                stateMachineFlow++;
                break;
            case 22:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 23:
                //turn 90 degrees to the left
                robot.gStatTurn(.5,90);
                stateMachineFlow++;
                break;
            case 24:
                //wait 5 seconds then stop
                while (5 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
        }
    }
}
