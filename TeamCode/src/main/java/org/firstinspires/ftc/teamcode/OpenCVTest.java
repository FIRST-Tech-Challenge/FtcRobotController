package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;
import org.opencv.core.*;

@Autonomous
public class OpenCVTest extends OpMode {

    /*
    This is a test program intended to help teach our team members Easy OpenCV.
     */

    /*
    This is a DetectRed internal class intended to detect whether the color red is in one rectangle
    or the other. All internal classes that are meant to do anything with EasyOpenCV must be a
    "pipeline" and extend OpenCVPipeline. They also have to include processFrame(Mat input) which
    is the actual image processing.
     */
    class DetectRed extends OpenCvPipeline {
        /*
        Mat variables represent specific images. The YCrCb variable represents the input image
        that the camera sends, changed to YCrCb color format. Cr represents the color's divergence
        from red, while Cb represents the color's divergence from blue. Y represents the color's
        brightness. leftCrop is a Mat that represents the left half of the image, while rightCrop
        represents the right half of the image. leftAverageFinal and rightAverageFinal respectively
        represent the left and right average red values of the image, and these will be ultimately
        compared to each other to determine if the red object is in the left or right half. The two
        RED and GREEN scalars are just for convenience so that I don't have to repeat the same
        RGB color values for both the left and right rectangles.
         */
        Mat YCrCb = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat output = new Mat();
        double leftAverageFinal;
        double rightAverageFinal;
        final Scalar RED = new Scalar(255, 0, 0);
        final Scalar GREEN = new Scalar(0, 255, 0);

        // This is the function that does the actual image processing. It takes in
        // the input frame from the camera, and it outputs the edited frame with rectangles along
        // with the code to determine whether the object is in the right or left.
        public Mat processFrame(Mat input) {

            /*
            Imgproc contains all the functions to edit the image. For cvtColor, that changes the
            image from one color format to another. In this case, we are changing from RGB to
            YCrCb. The first parameter is the input frame, the second is the output frame, and
            the third is the enum for what the image is being transferred from and to. The telemetry
            is just to ensure that the pipeline is running.
             */
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("DetectRed Pipeline Running");

            /*
            This is two rectangles that represent the left and right halves of the camera. The
            values are chosen because the resolution is 640x360, but if the resolution was changed
            this would need to be changed too.
             */
            Rect leftRect = new Rect(1, 1, 319, 359);
            Rect rightRect = new Rect(320, 1, 319, 359);

            /*
            As we already know, Imgproc edits the image. Imgproc.rectangle edits the image so that
            our previously defined rectangles are drawn in RED.
             */
            Imgproc.rectangle(output, leftRect, RED, 2);
            Imgproc.rectangle(output, rightRect, RED, 2);

            /*
            A submat is a piece of a Mat, aka a subsection of an image. This separates the YCrCb
            Mat, AKA the input frame turned into YCrCb image, into two Mats divided by the
            previously defined rectangles.
             */
            leftCrop = YCrCb.submat(leftRect);
            rightCrop = YCrCb.submat(rightRect);

            /*
            This function extracts one color channel from the image and puts it back into the image.
            The first parameter is the input, the second parameter is the output. Because they are
            the same here, this is effectively editing the image. The last parameter is the channel
            to extract from the image. In this case it is channel 2. In Y Cr Cb format, this channel
            is Cr, AKA the divergence from red.
             */
            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            // This Scalar is the mean color of the Mat that it is taking from.
            Scalar leftAverage = Core.mean(leftCrop);
            Scalar rightAverage = Core.mean(rightCrop);

            // This double is the final average values of the red in the submats, which is
            // equal to the first digit of the Scalar.
            leftAverageFinal = leftAverage.val[0];
            rightAverageFinal = rightAverage.val[0];

            // Finally, we compare the left and right average finals.
            // The correct rectangle will turn green. Telemetry will also be outputted.
            if (leftAverageFinal > rightAverageFinal) {
                Imgproc.rectangle(output, leftRect, GREEN, 2);
                telemetry.addLine("Red object detected in left.");
            }
            else if (leftAverageFinal < rightAverageFinal) {
                Imgproc.rectangle(output, rightRect, GREEN, 2);
                telemetry.addLine("Red object detected in right.");
            }
            else {
                telemetry.addLine("Red amount in both is equal.");
            }

            // This part just outputs the average red values in both to the telemetry.
            telemetry.addData("Left Average Red Value: ", leftAverageFinal);
            telemetry.addData("Right Average Red Value: ", rightAverageFinal);

            // This outputs the input Camera frame, edited to include the two rectangles.
            return output;
        }
    }

    public void init() {
        /*
        This entire part is boilerplate code that is generally copypastable. It's only purpose is
        to start a Camera pipeline and start the Camera's streaming.
        The only part that should not be copypasted is the camera.setPipeline, as that sets the
        wanted pipeline. This can also be done in the loop() function and multiple pipelines can be
        swapped between as the autonomous goes on.
        */

        WebcamName webcam = hardwareMap.get(WebcamName.class, "Camera");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);
        camera.setPipeline(new DetectRed());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                System.out.println("it no work");
            }
        });
    }

    @Override
    public void loop() {
        telemetry.update();
    }

}