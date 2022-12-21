package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")

public class AutonFinalHopefully extends LinearOpMode {
    
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private static final String TFOD_MODEL_ASSET = "STAcutomsleeve.tflite";
    private static final String[] LABELS = {
            "1",
            "2",
            "3"
    };
     
    private static final String VUFORIA_KEY =
            "AQ/k8Pn/////AAABmSy7KZgfyEe1kMc9qURPFXFlEtlhtT2tXuzZcSOF+pJ37wP+gbzpHfSyux1+a9nFER222NPE9imJD/PIHzXRa4/D4V4vjb1iCllEUvLGjyVL3DiNVI8mUqKxIUPjXc/zT447pD5yyu0qWHtfMju/1T7OZ+3ss5t/SOoVEw6M6X7kCUMNvxq9jhyESCfVyYD0o06n6UMJ+XPx0bI/9zuKCVmqaeH7UUh+aRr/a83Fu7EpZUZ1pY+o1QgcNBTWLOfIQJYsnqliL7RtHWUcNGqUKxHpfe28nrptQTg/EOc3xOmmIxZcmKGkkO5ykaL0R9iLmezR1P4qMDqEuSK/AJ9txtj0DARVgI/TomunzevewDiF";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    
    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }
        waitForStart();
            while (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                          

                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                            telemetry.update();
                            if(recognition.getLabel() == "1"){
                                
                            leftBack = hardwareMap.get(DcMotor.class, "leftBack");
                            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
                            rightBack = hardwareMap.get(DcMotor.class, "rightBack");
                            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            
                            leftBack.setDirection(DcMotor.Direction.FORWARD);
                            rightBack.setDirection(DcMotor.Direction.FORWARD);
                            leftFront.setDirection(DcMotor.Direction.FORWARD);
                            rightFront.setDirection(DcMotor.Direction.FORWARD);
        
                            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

                            telemetry.addData("Status", "Initialized");
                            telemetry.update();
                            waitForStart();
        
                            leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
                            rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
                            leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
                            rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
                            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
                            leftBack.setTargetPosition(1860);
                            rightBack.setTargetPosition(1860);
                            leftFront.setTargetPosition(-1860);
                            rightFront.setTargetPosition(-1860);
        
                            leftBack.setPower(0.5);
                            rightBack.setPower(0.5);
                            leftFront.setPower(0.5);
                            rightFront.setPower(0.5);
        
                            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
                                idle();
                            }
                             leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
                             rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
                             leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
                             rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
                            
                             leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                             rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                             leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                             rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                            
                             leftBack.setTargetPosition(1860);
                             rightBack.setTargetPosition(-1860);
                             leftFront.setTargetPosition(1860);
                             rightFront.setTargetPosition(-1860);
                            
                             leftBack.setPower(0.5);
                             rightBack.setPower(0.5);
                             leftFront.setPower(0.5);
                             rightFront.setPower(0.5);
                            
                             leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                             rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                             leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                             rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                 while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
                                     idle();
                                 }
                            }
                        }
                    }
                }
            }
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.65f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
}
