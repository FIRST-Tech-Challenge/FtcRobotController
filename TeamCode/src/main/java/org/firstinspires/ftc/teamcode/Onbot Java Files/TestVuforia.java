package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Vuforia Test", group = "TestCode")
@Disabled
public class TestVuforia extends LinearOpMode {

    private static final String VUFORIA_KEY = "AfT+h3P/////AAABmWYZVy7a/EcihirwYBQzqzOKAK+jNFQxAafHnrSltR+HuuxnWA43cR9KwSnaHKmK4nmR2Cnaplm5vI3SInFbPz3JGHLnwF+rTL4fa75cn0dXxlpou2oi7wxIeR2pTXDEjMlhlOB0Awfc/MSi02IN1srBisDq6nTlKL4aKdd+io1InJVKZmQe2qdIx/3JWDut21NdgKNQbuECPjTNQJPRf/fyzTRROnyN9RG2Bc3SZb14Rnh3aBbKYbzJ9EjmlXXtNHWPXw8UcHiwQbB9+x1FbbfzB8Wxj/4wb8Re1tkYb4keaQaofFA4/9tBuq+QuB9valPnCT18hQLz1W4y4brlDckvhK7wP1VlRXWCgPW62+oj";
    private VuforiaLocalizer vuforia;


    // todo: write your code here
    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        try {
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        } 
        
        catch (Exception e) {
            telemetry.addData("Exception", e ) ;
        }
        
        telemetry.addLine("Ok so far") ;
        telemetry.update() ;
        
        sleep (10000) ;
        

    }
    
}

