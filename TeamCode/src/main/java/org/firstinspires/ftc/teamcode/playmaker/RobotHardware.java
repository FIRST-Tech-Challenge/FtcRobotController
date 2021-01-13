package org.firstinspires.ftc.teamcode.playmaker;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.OmniDrive;

public abstract class RobotHardware extends OpMode {

    public enum ExecutionMode {
        TELEOP,
        HYBRID_OP,
        AUTONOMOUS
    }

    public enum Team {
        RED,
        BLUE
    }

    private ExecutionMode executionMode = ExecutionMode.TELEOP;

    public static final String vuforiaKey = "ActI1F//////AAABmS42p5yOnkGis4OjI6bXOlAnHWRg28DHHDgR3ja8s8s9yCGhUmk3wfLPYxAOtfsiSVSi97uAosw46Pu3KQNf7fSqrMOT/PUcG2zW3Lq8tnJHTe/uwhwWgvnwOlrgEovZPA0uhwQ/uHH2zr/U2mFMYOQTTAk6ovbCjARxN+HfP6XWCDHDQ4dhOK+joRlA8u0HqXPzm6uBQWBgCyUno8aESPLQu3QGgEWUWm1tEhUny4rgQXC19nH160f7EGy+YoTR6YAD37xQQxnzP58wHmrX7+cBuiwkai9+g65R3pfBYprNpeRunzEml6m+a792ypI/niKew1VWPSgQSHaE1Ix8+c6uCvqySjcu5mZ1g3/pnU2j";
    public VuforiaLocalizer.Parameters vuforiaParameters;
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    public WebcamName webcamName;
    public boolean initVuforia = true;
    public boolean initTfod = true;

    // Executors
    public AutonomousExecutor autonomousExecutor;
    public HybridOpExecutor hybridOpExecutor;

    public BNO055IMU revIMU;
    public OmniDrive omniDrive;
    public Localizer  localizer;

    public GamepadActions gamepadActions;

    /**
     * All hardware should initialize sensors and stuff here
     */
    public abstract void initializeHardware();

    public void initializeAutonomous() {}

    public void initializeTeleOp() {}

    public <T extends HardwareDevice> T initializeDevice(Class<? extends T> deviceClass, String name) {
        try {
            return this.hardwareMap.get(deviceClass, name);
        } catch (Exception e) {
            this.telemetry.addLine(String.format("Err: Device \"%s\" cannot be found.", name));
            return null;
        }
    }

    /**
     * Initializes Vuforia. Largely copied from the the navigation example.
     */
    public void initializeVuforia() {
        this.webcamName = this.initializeDevice(WebcamName.class,"Webcam 1");
        if (this.webcamName == null) {
            telemetry.addData("Robot Hardware", "VUFORIA INIT FAILED, WEBCAM NULL");
            return;
        }

        vuforiaParameters = new VuforiaLocalizer.Parameters();

//        try {
//            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//            vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        } catch (Exception e) {
//            vuforiaParameters = new VuforiaLocalizer.Parameters();
//        }

        vuforiaParameters.vuforiaLicenseKey = vuforiaKey;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        vuforiaParameters.cameraName = webcamName;
        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        // Make sure extended tracking is disabled for this example.
        vuforiaParameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);

    }

    public void initTfod() {
        TFObjectDetector.Parameters tfodParameters;
        try {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        } catch (Exception e) {
            tfodParameters = new TFObjectDetector.Parameters();
        }
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    }

    public void initializeLocalizer() {
        this.localizer = new Localizer();
    }

    public void initializeOmniDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.omniDrive = new OmniDrive(frontLeft, frontRight, backLeft, backRight);
    }

    @Override
    public void init() {
        if (initVuforia) initializeVuforia();
        if (initTfod) initTfod();
        this.initializeLocalizer();
        this.initializeHardware();
        this.gamepadActions = new GamepadActions();
    }

    /**
     * Prepares the hardware for autonomous execution;
     * @param autonomous Autonomous to be executed
     */
    public void initializeForAutonomous(Autonomous autonomous) {
        this.executionMode = ExecutionMode.AUTONOMOUS;
        this.autonomousExecutor = new AutonomousExecutor(autonomous, this);
    }

    /**
     * Prepares the hardware for hybrid op execution;
     * @param hybridOp HybridOp to be executed
     */
    public void initializeForHybridOp(HybridOp hybridOp) {
        this.executionMode = ExecutionMode.HYBRID_OP;
        this.hybridOpExecutor = new HybridOpExecutor(hybridOp, this);
    }


    public void hardware_loop() {}

    public abstract void run_loop();

    @Override
    public void loop() {
        // Update robot location
        this.localizer.updateRobotTransform(this, revIMU);

        // Update gamepad actions
        gamepadActions.update(gamepad1, gamepad2);

        switch (executionMode) {
            case TELEOP:
                break;
            case HYBRID_OP:
                this.hybridOpExecutor.loop();
                break;
            case AUTONOMOUS:
                boolean done = autonomousExecutor.loop();
                if (done) requestOpModeStop();
                break;
        }

        this.hardware_loop();
        this.run_loop();
    }
}
