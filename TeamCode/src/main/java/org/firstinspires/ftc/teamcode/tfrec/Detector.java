package org.firstinspires.ftc.teamcode.tfrec;

import android.app.Activity;
import android.app.Fragment;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Typeface;
import android.hardware.Camera;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Trace;
import android.util.Log;
import android.util.Size;
import android.util.TypedValue;
import android.view.Surface;
import android.view.View;
import android.widget.FrameLayout;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier.Device;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier.Model;
import org.firstinspires.ftc.teamcode.tfrec.utils.BorderedText;
import org.firstinspires.ftc.teamcode.tfrec.utils.ImageUtils;
import org.firstinspires.ftc.teamcode.tfrec.views.CameraConnectionFragment;
import org.firstinspires.ftc.teamcode.tfrec.views.LegacyCameraConnectionFragment;

import java.nio.ByteBuffer;
import java.util.List;

public class Detector implements ImageReader.OnImageAvailableListener, Camera.PreviewCallback {
    private static final String TAG = "Detector";
    private Bitmap rgbFrameBitmap = null;
    String cameraId;
    private Integer sensorOrientation;
    private BorderedText borderedText;
    private int[] rgbBytes = null;
    private byte[][] yuvBytes = new byte[3][];
    private Runnable imageConverter;
    private Runnable postInferenceCallback;
    private Handler handler;
    private HandlerThread handlerThread;
    protected int previewWidth = 0;
    protected int previewHeight = 0;
    private int yRowStride;
    Telemetry telemetry;
    boolean useCamera2API = false;
    private boolean isProcessingFrame = false;
    private Context appContext;
    private Classifier classifier;
    private Model model = Model.QUANTIZED_EFFICIENTNET;
    private Device device = Device.CPU;
    private String modelPath;
    private String labelPath;
    private int numThreads = 1;
    private static final float TEXT_SIZE_DIP = 10;
    private static final Size DESIRED_PREVIEW_SIZE = new Size(640, 480);
    private List<Classifier.Recognition> lastResults = null;
    private int tfodMonitorViewId = -1;
    private Fragment fragment;

    /** Input image size of the model along x axis. */
    private int imageSizeX;
    /** Input image size of the model along y axis. */
    private int imageSizeY;


    public Detector(Model modelType, String modelPath, Context ctx, Telemetry t) throws Exception{
        String modelFileName = modelPath.substring(0, modelPath.lastIndexOf('.'));
        String labelFileName = String.format("%s_labels.txt", modelFileName);
        modelFileName = String.format("%s.tflite", modelFileName);
        init(modelType, modelFileName, labelFileName, ctx, t);
    }

    public Detector(Model modelType, String modelPath, String labelPath, Context ctx, Telemetry t) throws Exception{
        init(modelType, modelPath, labelPath, ctx, t);
    }

    protected void init(Model modelType, String modelPath, String labelPath, Context ctx, Telemetry t) throws Exception{
        Log.d(TAG, "Classifier. Starting Init method");
        appContext = ctx;
        telemetry = t;
        tfodMonitorViewId = appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", appContext.getPackageName());
        try {
            ((Activity)appContext).runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    //make visible
                    FrameLayout fm = (FrameLayout)((Activity)appContext).findViewById(tfodMonitorViewId);
                    if (fm != null){
                        fm.setVisibility(View.VISIBLE);
                    }
                }
            });

            setModelType(modelType);
            setModelPath(modelPath);
            setLabelPath(labelPath);
            Log.d(TAG, "Classifier. Init method complete");
        } catch (Exception e) {
            Log.e(TAG, "Init method error", e);
            throw new Exception("Make frame visible", e);
        }
    }

    protected synchronized void startProcessing() {
        handlerThread = new HandlerThread("inference");
        handlerThread.start();
        handler = new Handler(handlerThread.getLooper());
    }

    public synchronized void stopProcessing() {
        if(handlerThread == null) {
            return;
        }
        handlerThread.quitSafely();
        try {
            handlerThread.join();
            handlerThread = null;
            handler = null;
            if (fragment != null) {
                ((Activity) appContext).getFragmentManager().beginTransaction().remove(fragment).commit();
            }

        } catch (final InterruptedException e) {
            Log.e(TAG, "Unable to stop processing", e);
        }
    }


    public void activate() throws Exception{
        startProcessing();
        final CameraManager manager = (CameraManager) appContext.getSystemService(Context.CAMERA_SERVICE);
        try {
            for (final String cameraId : manager.getCameraIdList()) {
                final CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);
                Log.d(TAG, String.format("Activation. Cam ID: %s", cameraId));

                final Integer facing = characteristics.get(CameraCharacteristics.LENS_FACING);
                Log.d(TAG, String.format("Activation. Facing: %d", facing));

                Integer orientation = characteristics.get(CameraCharacteristics.SENSOR_ORIENTATION);
                Log.d(TAG, String.format("Activation. Orientation: %d", orientation));

                if (facing != null && facing == CameraCharacteristics.LENS_FACING_BACK) {

                    final StreamConfigurationMap map =
                            characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);

                    if (map == null) {
                        continue;
                    }

                    // Fallback to camera1 API for internal cameras that don't have full support.
                    // This should help with legacy situations where using the camera2 API causes
                    // distorted or otherwise broken previews.
                    useCamera2API = (facing == CameraCharacteristics.LENS_FACING_EXTERNAL)
                                    || isHardwareLevelSupported(
                                    characteristics, CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_FULL);
                    Log.d(TAG, String.format("Activation. Camera API lv2?: %b", useCamera2API));
                    this.cameraId = cameraId;
                    break;
                }
            }
            Log.d(TAG, String.format("Activation. Selected Camera: %s", cameraId));
            setFragment();
            Log.d(TAG, "Activation. Complete");
        } catch (CameraAccessException e) {
            Log.e(TAG, "Not allowed to access camera", e);
            throw new Exception("Not allowed to access camera", e);
        }
        catch (Exception e) {
            Log.e(TAG, "Problems with activation", e);
            throw new Exception("Problems with activation", e);
        }
    }

    protected void setFragment() {
        if (useCamera2API) {
            CameraConnectionFragment camera2Fragment =
                    CameraConnectionFragment.newInstance(
                            new CameraConnectionFragment.ConnectionCallback() {
                                @Override
                                public void onPreviewSizeChosen(final Size size, final int rotation) {
                                    previewHeight = size.getHeight();
                                    previewWidth = size.getWidth();
                                    Detector.this.onPreviewSizeChosen(size, rotation);
                                }
                            },
                            this,
                            getLayoutId(),
                            getDesiredPreviewFrameSize(), telemetry);

            camera2Fragment.setCamera(this.cameraId);
            fragment = camera2Fragment;
        } else {
            fragment =
                    new LegacyCameraConnectionFragment(this, getLayoutId(), getDesiredPreviewFrameSize(), telemetry);
        }


        ((Activity)appContext).getFragmentManager().beginTransaction().replace(tfodMonitorViewId, fragment).commit();
        Log.d(TAG, "SetFragment. Complete");
    }

    protected int getLayoutId() {
        return R.layout.tfe_ic_camera_connection_fragment;
    }

    protected Size getDesiredPreviewFrameSize() {
        return DESIRED_PREVIEW_SIZE;
    }

    private boolean isHardwareLevelSupported(
            CameraCharacteristics characteristics, int requiredLevel) {
        int deviceLevel = characteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL);
        if (deviceLevel == CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY) {
            return requiredLevel == deviceLevel;
        }
        // deviceLevel is not LEGACY, can use numerical sort
        return requiredLevel <= deviceLevel;
    }


    protected int[] getRgbBytes() {
        imageConverter.run();
        return rgbBytes;
    }

    protected void fillBytes(final Image.Plane[] planes, final byte[][] yuvBytes) {
        // Because of the variable row stride it's not possible to know in
        // advance the actual necessary dimensions of the yuv planes.
        for (int i = 0; i < planes.length; ++i) {
            final ByteBuffer buffer = planes[i].getBuffer();
            if (yuvBytes[i] == null) {
                yuvBytes[i] = new byte[buffer.capacity()];
            }
            buffer.get(yuvBytes[i]);
        }
    }


    protected void readyForNextImage() {
        if (postInferenceCallback != null) {
            postInferenceCallback.run();
        }
    }

    protected int getScreenOrientation() {
        switch (((Activity)appContext).getWindowManager().getDefaultDisplay().getRotation()) {
            case Surface.ROTATION_270:
                return 270;
            case Surface.ROTATION_180:
                return 180;
            case Surface.ROTATION_90:
                return 90;
            default:
                return 0;
        }
    }

    protected synchronized void runInBackground(final Runnable r) {
        if (handler != null) {
            handler.post(r);
        }
    }

    @Override
    public void onPreviewFrame(final byte[] bytes, final Camera camera) {
        if (isProcessingFrame) {
            Log.d(TAG, "onPreviewFrame. Dropping frame");
            return;
        }

        try {
            // Initialize the storage bitmaps once when the resolution is known.
            if (rgbBytes == null) {
                Camera.Size previewSize = camera.getParameters().getPreviewSize();
                previewHeight = previewSize.height;
                previewWidth = previewSize.width;
                rgbBytes = new int[previewWidth * previewHeight];
                onPreviewSizeChosen(new Size(previewSize.width, previewSize.height), 90);
            }
        } catch (final Exception e) {
            Log.e(TAG, "onPreviewFrame. Error", e);
            return;
        }

        if (classifier == null) {
            Log.e(TAG, "No classifier on preview!");
            return;
        }

        isProcessingFrame = true;
        yuvBytes[0] = bytes;
        yRowStride = previewWidth;

        imageConverter =
                new Runnable() {
                    @Override
                    public void run() {
                        ImageUtils.convertYUV420SPToARGB8888(bytes, previewWidth, previewHeight, rgbBytes);
                    }
                };

        postInferenceCallback =
                new Runnable() {
                    @Override
                    public void run() {
                        camera.addCallbackBuffer(bytes);
                        isProcessingFrame = false;
                    }
                };
        processImage();
    }

    public void onPreviewSizeChosen(final Size size, final int rotation) {
        final float textSizePx =
                TypedValue.applyDimension(
                        TypedValue.COMPLEX_UNIT_DIP, TEXT_SIZE_DIP, appContext.getResources().getDisplayMetrics());
        borderedText = new BorderedText(textSizePx);
        borderedText.setTypeface(Typeface.MONOSPACE);

        recreateClassifier(getModelType(), getDevice(), getNumThreads());
        if (classifier == null) {
            Log.e(TAG, "onPreviewSizeChosen. No classifier on preview!");
            return;
        }

        previewWidth = size.getWidth();
        previewHeight = size.getHeight();

        sensorOrientation = rotation - getScreenOrientation();

        Log.d(TAG, String.format("onPreviewSizeChosen. Camera orientation relative to screen canvas: %d", sensorOrientation));
        Log.d(TAG, String.format("onPreviewSizeChosen. Initializing at size %dx%ds:", previewWidth, previewHeight));
        rgbFrameBitmap = Bitmap.createBitmap(previewWidth, previewHeight, Bitmap.Config.ARGB_8888);
    }

    protected void processImage() {
        rgbFrameBitmap.setPixels(getRgbBytes(), 0, previewWidth, 0, 0, previewWidth, previewHeight);
//        final int cropSize = Math.min(previewWidth, previewHeight);

        runInBackground(
                new Runnable() {
                    @Override
                    public void run() {
                        if (classifier != null) {
                            lastResults = classifier.recognizeImage(rgbFrameBitmap, sensorOrientation);
                        }
                        else{
                            Log.e(TAG, "processImage. No Classifier to process image");
                        }
                        readyForNextImage();
                    }
                });
    }

    @Override
    public void onImageAvailable(ImageReader imageReader) {
        if (previewWidth == 0 || previewHeight == 0) {
            return;
        }
        if (rgbBytes == null) {
            rgbBytes = new int[previewWidth * previewHeight];
        }
        try {
            final Image image = imageReader.acquireLatestImage();

            if (image == null) {
                return;
            }

            if (isProcessingFrame) {
                image.close();
                return;
            }
            isProcessingFrame = true;
            Trace.beginSection("imageAvailable");
            final Image.Plane[] planes = image.getPlanes();
            fillBytes(planes, yuvBytes);
            yRowStride = planes[0].getRowStride();
            final int uvRowStride = planes[1].getRowStride();
            final int uvPixelStride = planes[1].getPixelStride();

            imageConverter =
                    new Runnable() {
                        @Override
                        public void run() {
                            ImageUtils.convertYUV420ToARGB8888(
                                    yuvBytes[0],
                                    yuvBytes[1],
                                    yuvBytes[2],
                                    previewWidth,
                                    previewHeight,
                                    yRowStride,
                                    uvRowStride,
                                    uvPixelStride,
                                    rgbBytes);
                        }
                    };

            postInferenceCallback =
                    new Runnable() {
                        @Override
                        public void run() {
                            image.close();
                            isProcessingFrame = false;
                        }
                    };

            processImage();
        } catch (final Exception e) {
            Log.e(TAG, "onImageAvailable error", e);
            Trace.endSection();
            return;
        }
        Trace.endSection();
    }

    protected void onInferenceConfigurationChanged() {
        if (rgbFrameBitmap == null) {
            // Defer creation until we're getting camera frames.
            return;
        }
        final Device device = getDevice();
        final Model model = getModelType();
        final int numThreads = getNumThreads();
        runInBackground(new Runnable() {
            @Override
            public void run() {
                Detector.this.recreateClassifier(model, device, numThreads);
            }
        });
    }

    private void recreateClassifier(Model model, Device device, int numThreads) {
        if (classifier != null) {
            Log.d(TAG, "recreateClassifier. Closing classifier");
            classifier.close();
            classifier = null;
        }
        if (device == Device.GPU
                && (model == Model.QUANTIZED_MOBILENET || model == Model.QUANTIZED_EFFICIENTNET)) {
            Log.e(TAG, "recreateClassifier. Not creating classifier: GPU doesn't support quantized models.");
            return;
        }
        try {
            classifier = Classifier.create((Activity)this.appContext, model, device, numThreads, this.getModelPath(), this.getLabelPath(), telemetry);
            Log.d(TAG, String.format("Created classifier (model=%s, device=%s, numThreads=%d)", model, device, numThreads));
        }
        catch (Exception e) {
            Log.e(TAG, "Failed to create classifier", e);
        }

        if (classifier != null) {
            // Updates the input image size.
            imageSizeX = classifier.getImageSizeX();
            imageSizeY = classifier.getImageSizeY();
            Log.d(TAG, "Classifier done");
        }
        else{
            Log.e(TAG, "Failed to create classifier");
        }
    }

    protected Model getModelType() {
        return model;
    }

    public void setModelType(Model model) {
        if (this.model != model) {
            this.model = model;
            Log.d(TAG, String.format("Updated model %s", model));
//            onInferenceConfigurationChanged();
        }
    }

    protected Device getDevice() {
        return device;
    }


    private void setDevice(Device device) {
        if (this.device != device) {
            this.device = device;
            Log.d(TAG, String.format("Updated device %s", device));
            final boolean threadsEnabled = device == Device.CPU;
            onInferenceConfigurationChanged();
        }
    }

    protected int getNumThreads() {
        return numThreads;
    }

    private void setNumThreads(int numThreads) {
        if (this.numThreads != numThreads) {
            this.numThreads = numThreads;
            onInferenceConfigurationChanged();
        }
    }

    public List<Classifier.Recognition> getLastResults() {
        return lastResults;
    }

    public String getModelPath() {
        return modelPath;
    }

    public void setModelPath(String modelPath) {
        this.modelPath = modelPath;
    }

    public String getLabelPath() {
        return labelPath;
    }

    public void setLabelPath(String labelPath) {
        this.labelPath = labelPath;
    }
}
