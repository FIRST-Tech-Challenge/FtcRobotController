package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp
public class SampleDetectorV2 extends OpMode {
    public SampleDetectorToolkit toolkit;
    public SampleDetectorVisionPortalToolkit visionToolkit;
    public ColorBlobLocatorProcessor red;
    public ColorBlobLocatorProcessor neutral;
    public ColorBlobLocatorProcessor blue;
    public List<ColorBlobLocatorProcessor> processorList = new ArrayList<>();
    public List<ColorBlobLocatorProcessor.Blob> redList = new ArrayList<>();
    public List<ColorBlobLocatorProcessor.Blob> neutralList = new ArrayList<>();
    public List<ColorBlobLocatorProcessor.Blob> blueList = new ArrayList<>();
    public List<SampleDetection> masterList = new ArrayList<>();
    public VisionPortal portal;
    public int MIN_AREA = 2000;
    public int MAX_AREA = 10000;
    public Point center;
    public ElapsedTime buttonTimer = new ElapsedTime();
    public enum SelectedColors {
        RED_ONLY,
        BLUE_ONLY,
        YELLOW_ONLY,
        RED_AND_BLUE,
        RED_AND_YELLOW,
        BLUE_AND_YELLOW,
        ALL
    }
    public int index = 0;
    public SelectedColors[] modesList = {SelectedColors.RED_ONLY, SelectedColors.YELLOW_ONLY, SelectedColors.RED_AND_BLUE, SelectedColors.RED_AND_YELLOW, SelectedColors.BLUE_AND_YELLOW, SelectedColors.ALL};
    public boolean selected = false;
    public SelectedColors selectedColors;
    @Override
    public void init() {
        selectedColors = SelectedColors.RED_ONLY;
        telemetry.setMsTransmissionInterval(50);
        toolkit = new SampleDetectorToolkit(hardwareMap);
        visionToolkit = new SampleDetectorVisionPortalToolkit(hardwareMap);
        center = toolkit.getCenter(320, 240);
        buttonTimer.reset();
    }

    @Override
    public void init_loop() {
        // Handle user selection of color modes
        if (gamepad1.dpad_right && buttonTimer.seconds() > 0.3) {
            index = Math.min(index + 1, modesList.length - 1);
            buttonTimer.reset();
        } else if (gamepad1.dpad_left && buttonTimer.seconds() > 0.3) {
            index = Math.max(index - 1, 0);
            buttonTimer.reset();
        }

        selectedColors = modesList[index];
        telemetry.addLine("Use D-Pad to change modes");
        telemetry.addLine("Selected mode: " + selectedColors.name());
        telemetry.addLine("Press 'A' to confirm selection");
        telemetry.update();

        // If user confirms selection, start processors
        if (gamepad1.a) {
            selected = true;
            configureAndStartProcessors(selectedColors);
        }
    }


    private void configureAndStartProcessors(SelectedColors selectedColors) {
        // Clear the processor list and configure based on the selected mode
        processorList.clear();
        switch (selectedColors) {
            case RED_ONLY:
                addProcessor(ColorRange.RED);
                break;
            case YELLOW_ONLY:
                addProcessor(ColorRange.YELLOW);
                break;
            case BLUE_ONLY:
                addProcessor(ColorRange.BLUE);
                break;
            case RED_AND_BLUE:
                addProcessor(ColorRange.RED);
                addProcessor(ColorRange.BLUE);
                break;
            case RED_AND_YELLOW:
                addProcessor(ColorRange.RED);
                addProcessor(ColorRange.YELLOW);
                break;
            case BLUE_AND_YELLOW:
                addProcessor(ColorRange.BLUE);
                addProcessor(ColorRange.YELLOW);
                break;
            case ALL:
                addProcessor(ColorRange.RED);
                addProcessor(ColorRange.YELLOW);
                addProcessor(ColorRange.BLUE);
                break;
        }

        // Start the vision portal with the configured processors
        portal = visionToolkit.createVisionPortal(hardwareMap, processorList, "Webcam 1");
    }

    private void addProcessor(ColorRange colorRange) {
        ColorBlobLocatorProcessor processor = toolkit.getNewProcessor(colorRange, MIN_AREA, MAX_AREA);
        processorList.add(processor);
    }


    @Override
    public void loop() {
        portal.stopStreaming();
    }
}
