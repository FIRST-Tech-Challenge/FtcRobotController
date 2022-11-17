package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class Camera extends Control{
    Pipeline pipeline;


    @Override
    public void init(){
        super.init();
        hraezlyr.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // if crashing in libgls.so delete this line
                hraezlyr.camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

                hraezlyr.camera.setPipeline(pipeline);
                //            HOPE THIS WORKS ^ (if not ask tom)

                hraezlyr.camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);


            }

            @Override
            public void onError(int errorCode) {

            }

            @Override
            public boolean equals(@Nullable Object obj) {
                return super.equals(obj);
            }


        }
    }
}