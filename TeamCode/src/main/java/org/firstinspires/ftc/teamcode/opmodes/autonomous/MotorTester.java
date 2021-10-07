//package org.firstinspires.ftc.teamcode.opmodes.autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.R;
//import org.firstinspires.ftc.teamcode.utils.motors.ComplexMotorController;
//import org.firstinspires.ftc.teamcode.utils.motors.Motor;
//import org.firstinspires.ftc.teamcode.utils.motors.SimpleMotorController;
//
//@Autonomous(name="MotorTester", group="Iterative")
//@Disabled
//public class MotorTester extends OpMode {
//
//    private ComplexMotorController complexMotorController;
//    private Motor complexMotor;
//    private Motor[] simpleMotors;
//    private SimpleMotorController simpleMotorController;
//
//    /**
//     * Code to run once when the OpMode is initialized.
//     */
//    @Override
//    public void init() {
//        telemetry.addData("Status", "Initialized");
//    }
//
//    /*
//     * Code to loop between the end of init() and beginning of start().
//     */
//    @Override
//    public void init_loop() {
//
//    }
//
//    /*
//     * Code to run after hitting play.
//     */
//    @Override
//    public void start() {
//        TetrixMotor rightMotor = new TetrixMotor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.RIGHT_DRIVE_1), DcMotorSimple.Direction.FORWARD);
//        TetrixMotor leftMotor = new TetrixMotor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.LEFT_DRIVE_1), DcMotorSimple.Direction.REVERSE);
//        SimpleMotorController simpleMotorController = new SimpleMotorController(telemetry);
////        simpleMotorController.moveGroupedMotors(new TetrixMotor[]{rightMotor, leftMotor}, 12, 1, 5);
////        simpleMotorController.moveMultipleMotors(new TetrixMotor[]{rightMotor, leftMotor}, new double[]{12, -12}, 1, 5);
////        simpleMotorController.moveSingleMotor(rightMotor, 12, 1, 5);
//        ComplexMotorController complexMotorController = new ComplexMotorController(telemetry);
//        this.complexMotorController = complexMotorController;
//        this.complexMotor = leftMotor;
//        this.simpleMotorController = simpleMotorController;
//        this.simpleMotors = new TetrixMotor[]{rightMotor, leftMotor};
////        complexMotorController.prepMotor(leftMotor, 6);
////        complexMotorController.startMotor(leftMotor, 1);
////        complexMotorController.stopMotor(complexMotor);
//    }
//
//    /*
//     * Code to run after start() ends.
//     */
//    @Override
//    public void loop() {
//        simpleMotorController.moveGroupedMotors(simpleMotors, 999999, 1, 5);
//    }
//
//    /*
//     * Code to run once stop is pressed, or once the time runs out.
//     */
//    @Override
//    public void stop() {
//        simpleMotorController.killMotors(simpleMotors);
//    }
//
//}
