package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class Camera extends Control{
    Haezler haezler = new Haezler(hardwareMap);
    Pipeline pipeline;

    @Override
    public void init(){
        super.init();
        haezler.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // if crashing in libgls.so delete this line
                haezler.camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

                haezler.camera.setPipeline(pipeline);
                //            HOPE THIS WORKS ^ (if not ask tom)
                haezler.camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);



            }
            @Override
            public void onError(int errorCode)
            {

            }
        });
    }
}