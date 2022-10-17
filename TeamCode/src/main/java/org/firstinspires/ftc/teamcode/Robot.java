package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    protected Telemetry telemetry;
    protected FtcDashboard dashboard;
    protected Telemetry dashboardTelemetry;

    protected ExtendedGamepad gamepad1;
    protected ExtendedGamepad gamepad2;

    protected IMU gyro;
    protected MecanumDrivetrain drivetrain;
    protected HeadingController gyroFollow;

    protected Camera camera;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        /***************************** Setup and Initialize Gamepads ******************************/
        this.gamepad1 = new ExtendedGamepad(gamepad1);
        this.gamepad2 = new ExtendedGamepad(gamepad2);

        /******************************** Initialize FTC Dashboard ********************************/
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        this.telemetry = telemetry;

        /******************************** Camera ********************************/
        camera = new Camera(hardwareMap, dashboard, telemetry, gamepad1);

        /************************ Setup and Initialize Mechanisms Objects *************************/
        initMechanisms(hardwareMap);

        gyro = new IMU(hardwareMap, telemetry, dashboardTelemetry);
        drivetrain = new MecanumDrivetrain(hardwareMap);

        // If camera-follow then HeadingController.Type.CAMERA - Not Yet Implemented
        gyroFollow = new HeadingController(this.gamepad1, gyro);
    }

//    public double[] transformFieldXY2RobotXY(double xG, double yG) {
//        double heading = -gyro.getRawValue() * Math.PI / 180;
//        double[] robotXY = {
//                Math.cos(heading) * xG - Math.sin(heading) * yG,
//                Math.sin(heading) * xG + Math.cos(heading) * yG
//        };
//
//        return robotXY;
//    }

    public void update() {
        gamepad1.update();
        gamepad2.update();
        gyro.update();
        gyroFollow.update();
    }

    public void initMechanisms(HardwareMap hardwareMap) {
        // should be overridden by child class
    }

    public void runMechanisms() {
        // should be overridden by child class
    }

    public void run() {
        double heading = -gyro.getRawValue();
        double rot;

        if(gyroFollow.isEnabled()) {
            rot = gyroFollow.calculateTurn();
        } else if(camera.isEnabled()){
            rot = camera.getPipeline().getElementsAnalogCoordinates()[0];
        } else {
            rot = gamepad1.right_stick_x;
        }
//        double rot = -gamepad1.right_stick_x;
        drivetrain.run(gamepad1.left_stick_x, gamepad1.left_stick_y, rot, heading,
                gamepad1.right_trigger);

        runMechanisms();
    }

    public void telemetryUpdate() {
        telemetry.update();
    }

    public void dashboardTelemetryUpdate() {
        dashboardTelemetry.update();
    }
}