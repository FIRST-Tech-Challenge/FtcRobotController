package org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.Camera;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

@Deprecated
public class Vuforia extends VuforiaLocalizerImpl {

    public static final String VUFORIA_KEY = "AWVWPbH/////AAABmbzQF0cF/EvRnE4ykZKAXvpbnJrPQs1aBJ2i7u5ADGzYU+x0dxqGlB/G8yCrcY4FP8cPEA1w+xTXCpbFDmlYcKMG6VL/6v+H0Es3H/1f8xpQG86nSCXKPLxEbYGHkBxAYSlxB0gueBpnxMYsURezlq2Q9e5Br5OIhY7gmZZNa3VPHupscQkrCrVdRMI9mPAbEjMBhVBWjVJEL0+u2tyvEQuK4tllgi8C7AKq5V5lFoKEQG0VD89xlgUfRZsDq89HToRXBOUE2mubPHUcplKiX+1EfB+801eEt+k7lLJ1VyfrXr2tjwyWPjafvTpnaf3C35ox0/TOPdak5pq2gXLpXzAxXc6+RH28m2572tYB58AN";
    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters VFparameters;

    public Vuforia(VuforiaLocalizer.Parameters parameters) {
        super(parameters);
        VFparameters = parameters;
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */


        VFparameters.vuforiaLicenseKey = VUFORIA_KEY;
        VFparameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
}
