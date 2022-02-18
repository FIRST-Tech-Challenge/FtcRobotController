//package org.firstinspires.ftc.teamcode.Visual;
//
//import android.Manifest;
//import android.annotation.TargetApi;
//import android.content.Context;
//import android.content.pm.PackageManager;
//
//import android.graphics.Bitmap;
//import android.graphics.BitmapFactory;
//import android.graphics.Color;
//import android.graphics.ImageFormat;
//import android.hardware.camera2.CameraAccessException;
//import android.hardware.camera2.CameraCaptureSession;
//import android.hardware.camera2.CameraCharacteristics;
//import android.hardware.camera2.CameraDevice;
//import android.hardware.camera2.CameraManager;
//import android.hardware.camera2.CameraMetadata;
//import android.hardware.camera2.CaptureRequest;
//import android.media.Image;
//import android.media.ImageReader;
//import android.os.Handler;
//import android.os.Looper;
//import android.view.TextureView;
//import android.view.View;
//import android.view.ViewGroup;
//import android.widget.ImageView;
//
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.android.util.Size;
//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//
//import java.io.File;
//import java.io.FileOutputStream;
//import java.io.FilenameFilter;
//import java.nio.ByteBuffer;
//import java.util.Arrays;
//import java.util.Collections;
//import java.util.Objects;
//
//public class PhoneManager implements ImageReader.OnImageAvailableListener {
//    // A couple constants. Don't change unless you actually know what you're doing
//    private final Size FRAME_SIZE = new Size(1280, 720); // one of the supported frame sizes
//    private final int FPS = 6;  // = max fps
//
//    private boolean showsViews = false; // whether or not to show the views on the rc
//    private Context appContext; // the app context (from the hardwareMap)
//    private TextureView preview;
//
//    public boolean isNew = true;
//
//    private CameraDevice camera = null;
//    private CameraCaptureSession captureSession = null;
//    private Telemetry telemetry;
//
//    // Option to start the capture with views shown on the RC (needs appContext (hardwareMap.appContext) to display the views)
//    // ONLY do this if Vuforia is not in use. I'm not sure what that'll do since they use the same views.
//    public void startCaptureWithViews(Telemetry telemetry, Context appContext) {
//        this.appContext = appContext;
//        showsViews = true;                 // enable showing the views
//        initViews();                       // initialize the views
//        startCapture(telemetry, appContext);              // start the webcam capture
//    }
//
//
//    /**
//     * Start the webcam capture (if this is called directly, no views will be created on the RC)
//     */
//    @TargetApi(23)
//    // Only use this in android phones running 6.0.0 or newer OS. This overrides the RC's API level 19.
//    public void startCapture(final Telemetry telemetry, Context appContext) {
//        this.telemetry = telemetry;
//        this.appContext = appContext;
//        try {
//            CameraManager manager = (CameraManager) appContext.getSystemService(Context.CAMERA_SERVICE);
//            String cameraID = null;
//            for (String id : manager.getCameraIdList()) {
//                if (Objects.requireNonNull(manager.getCameraCharacteristics(id).get(CameraCharacteristics.LENS_FACING)) == CameraMetadata.LENS_FACING_BACK) {
//                    cameraID = id;
//                    break;
//                }
//            }
//
//            cameraID = Objects.requireNonNull(cameraID);
//
//            telemetry.addLine(Arrays.toString(manager.getCameraCharacteristics(cameraID).get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP).getOutputFormats()));
//            telemetry.addLine(Arrays.toString(manager.getCameraCharacteristics(cameraID).get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP).getOutputSizes(ImageFormat.JPEG)));
//
//            if (appContext.checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
//                telemetry.addLine("Missing camera permissions! Failed to open camera!");
//                return;
//            }
//
//            manager.openCamera(cameraID, new CameraDevice.StateCallback() {
//                @Override
//                public void onOpened(CameraDevice cameraDevice) {
//                    camera = cameraDevice;
//                    final ImageReader reader = ImageReader.newInstance(FRAME_SIZE.getWidth(), FRAME_SIZE.getHeight(), ImageFormat.JPEG, 1);
//                    reader.setOnImageAvailableListener(PhoneManager.this, new Handler(Looper.getMainLooper()));
//                    try {
//                        camera.createCaptureSession(Collections.singletonList(reader.getSurface()), new CameraCaptureSession.StateCallback() {
//                            @Override
//                            public void onConfigured(CameraCaptureSession cameraCaptureSession) {
//                                captureSession = cameraCaptureSession;
//                                telemetry.addLine("CONFIGURED!!!!");
//                                try {
//                                    CaptureRequest.Builder builder = camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
//                                    builder.addTarget(reader.getSurface());
//                                    builder.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
//                                    builder.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON);
//                                    builder.set(CaptureRequest.CONTROL_AWB_MODE, CaptureRequest.CONTROL_AWB_MODE_AUTO);
//                                    builder.set(CaptureRequest.JPEG_ORIENTATION, 180); // degrees
//                                    cameraCaptureSession.setRepeatingRequest(builder.build(), null, new Handler(Looper.getMainLooper()));
//                                } catch (CameraAccessException e) {
//                                    e.printStackTrace();
//                                }
//                            }
//
//                            @Override
//                            public void onConfigureFailed(CameraCaptureSession cameraCaptureSession) {
//                                telemetry.addLine("NOT CONFIGURED!!!!");
//                            }
//                        }, new Handler(Looper.getMainLooper()));
//                    } catch (CameraAccessException e) {
//                        e.printStackTrace();
//                    }
//
//                }
//
//                @Override
//                public void onDisconnected(CameraDevice cameraDevice) {
//                    telemetry.addLine("Hellooo");
//                }
//
//                @Override
//                public void onError(CameraDevice cameraDevice, int i) {
//                    telemetry.addLine("Error!");
//                }
//            }, new Handler(Looper.getMainLooper()));
//            telemetry.addLine("Hello");
//
//
//
//        } catch (CameraAccessException | NullPointerException e) {
//            e.printStackTrace();
//        }
//    }
//
//    // The bitmap in which to save the current frame. Must create it the same size as the frame recieved from the camera
//    //   Use format ARGB_8888 because that's the only one supported for conversion from YUY2 format which is all the webcam supports.
//    //   volatile means it can be edited from another thread and is not guaranteed to be the same as the code runs
//    private volatile Bitmap currentFrame = Bitmap.createBitmap(FRAME_SIZE.getWidth(), FRAME_SIZE.getHeight(), Bitmap.Config.ARGB_8888);
//
//    /**
//     * Get the current frame
//     * @return a copy of the current frame (since it's volatile and currentFrame could change as we're reading it later if we used it directly)
//     */
//    public Bitmap getCurrentFrame() {
//        // Use ARGB_8888 to be consistent with the frame format
//        return currentFrame.copy(Bitmap.Config.ARGB_8888, true);
//    }
//
//    @Override
//    public void onImageAvailable(ImageReader imageReader) {
//        Image image = imageReader.acquireLatestImage();
//        ByteBuffer buffer = image.getPlanes()[0].getBuffer();
//        byte[] bytes = new byte[buffer.capacity()];
//        buffer.get(bytes);
//        currentFrame = BitmapFactory.decodeByteArray(bytes, 0, bytes.length);// The bitmap is ARGB_8888
//        //telemetry.addLine(currentFrame.getConfig().name());
//        isNew = true;
//
//
//        // If we need to show the view,
//        if (showsViews)
//            showBitmap(cameraView, currentFrame); // then update it
//
//        // If we need to save the frames
//        if (shouldSaveFrames)
//            saveFrame(currentFrame); // then save it.
//        image.close();
//    }
//
//    public void stopCapture() {
//        if (captureSession != null)
//            captureSession.close();
//        cleanUpViews();
//        camera.close();
//    }
//
//    /**** Views ****/
//
//    private ViewGroup parentView; // The parent view which holds the two image views
//    private ImageView cameraView; // The ImageView which will hold the current camera frame
//    private ImageView userView; // The ImageView which will hold a user-supplied camera frame
//
//    /**
//     * Initialize the views!
//     * This is a bit complicated since I had to hack together a UI without the UI builder or any android UI experience.
//     */
//    private void initViews() {
//        // First check if we're really supposed to be doing this.
//        if (showsViews) {
//            // UI commands can only be called from the UI thread.
//            // Nasty errors ensue if you try from the normal thread.
//            // So we submit a Runnable to be run on the UI thread.
//            AppUtil.getInstance().synchronousRunOnUiThread(new Runnable() {
//                @Override
//                public void run() {
//                    try {
//                        // Get the parent View (this is the view Vuforia uses to display its preview)
//                        // The activity (current UI "page" on the phone) could be null, so make sure it's not (Objects.requireNonNull) or throw an error (caught with the try)
//                        // Find the view in the current activity by its id we can find from the hardwareMap's appContext.
//                        // This is why we need the appContext to make the views
//                        // The resource name is "cameraMonitorViewId", the same as with Vuforia
//                        parentView = (ViewGroup) Objects.requireNonNull(AppUtil.getInstance().getActivity()).findViewById(appContext.getResources().getIdentifier("cameraMonitorViewId", "id", appContext.getPackageName()));
//
//                        RobotLog.d("Found Parent View!" + parentView.toString()); // logging
//
//                        // Create an image view in the context of this application
//                        //  (I'm not sure why android makes me say it's for this application because um, if I'm making
//                        //  it in this application it's probably for this application, but oh well
//                        cameraView = new ImageView(AppUtil.getInstance().getApplication().getApplicationContext());
//
//                        // Create an empty (green) bitmap as a placeholder until actual frames are inserted into the views
//                        // I arbitrarily chose 1000x600 pixels because it seemed to fit.
//                        // Make sure you change it elsewhere (in showBitmap) if it's changed here.
//                        // Use ARGB_8888 for consistency
//                        Bitmap image = Bitmap.createBitmap(1000, 600, Bitmap.Config.ARGB_8888);
//                        image.eraseColor(Color.GREEN);     // fill it with green
//
//                        cameraView.setImageBitmap(image);  // set the image of the image view to the bitmap placeholder image
//
//                        parentView.addView(cameraView);    // add the camera view to the parent view
//
//                        RobotLog.d("Created Camera View"); // logging
//
//                        // Scale and rotate it (I'm not sure why I did this, but it probably makes it fit better on screen)
//                        cameraView.setScaleType(ImageView.ScaleType.CENTER_INSIDE);
//                        cameraView.setRotation(-90);
//
//
//                        // Make the other view exactly the same way
//                        userView = new ImageView(AppUtil.getInstance().getApplication().getApplicationContext());
//                        userView.setImageBitmap(image);
//                        parentView.addView(userView);
//                        userView.setScaleType(ImageView.ScaleType.CENTER_INSIDE);
//                        userView.setRotation(-90);
//                        RobotLog.d("Created Result View!", "");
//
//                        // Finally, now that all the views are added and initialized, we can show the parent view
//                        // (it starts invisible)
//                        parentView.setVisibility(View.VISIBLE);
//                    } catch (NullPointerException e) {
//                        // The activity was null for some reason... That's weird
//                        RobotLog.d("ERROR! NULL POINTER EXCEPTION");
//                        RobotLog.d(e.getMessage());
//                    }
//                }
//            });
//        }
//
//    }
//
//    /**
//     * Remove the views so we don't add multiple to the screen and so other programs can run
//     */
//    private void cleanUpViews() {
//        if (showsViews) {
//            AppUtil.getInstance().synchronousRunOnUiThread(new Runnable() {
//                @Override
//                public void run() {
//                    parentView.removeView(cameraView);
//                    parentView.removeView(userView);
//                }
//            });
//        }
//    }
//
//    /**
//     * Update the bitmap of an image view previously prepared. Intended to be used internally.
//     * The params must be final since they're being passed to an inner class.
//     *
//     * @param view either cameraView or userView
//     * @param bitmap the bitmap to update the image view with
//     */
//    private void showBitmap(final ImageView view, final Bitmap bitmap) {
//        // remember UI changes must be on the UI thread
//        AppUtil.getInstance().synchronousRunOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                // set the image in the image view to a scaled bitmap
//                // This keeps the image view from changing size and looking weird.
//                view.setImageBitmap(Bitmap.createScaledBitmap(bitmap, 1000, 600, false));
//            }
//        });
//    }
//
//
//    /**
//     * Public method to update the user-supplied bitmap.
//     * This is used for debugging to show an intermediate bitmap image after filtering or processing.
//     * @param bitmap the bitmap to show
//     */
//    public void updatePreviewBitmap(final Bitmap bitmap) {
//        if (showsViews) // make sure we've actually initialized the views and are ready for a bitmap
//            showBitmap(userView, bitmap); // show it!
//    }
//
//
//
//
//
//
//    /**** SAVE IMAGES ****/
//
//    private int framesToSkip = 5;              // The number of frames to skip in between saving (we probably don't want to save every frame)
//    private int framesSaved = 0;               // The total number of frames saved (so we can increment the file names and keep track of if we should save this one or not
//    private int userFramesSaved = 0;           // The total number of user frames saved (for incrementing file names)
//    private boolean shouldSaveFrames = false;  // Whether or not we should save frames (depends upon folders existing and if anyone's told us to start saving the frames
//    private File folder;                       // the root folder to save the frames in
//    private File originals;                    // the subfolder to put the original frames in
//    private File userSupplied;                 // the subfolder to put processed user frames in
//
//    /**
//     * Start saving the frames from now on to the folder.
//     * @param framesToSkip the number of frames to skip between saving the frames. I don't suggest 0.
//     * @return true if it succeeded in starting, false if it failed.
//     */
//    public void startSavingImages(int framesToSkip) {
//        initFolder(); // Should I save the frames? Only if the folders are ready for them.
//        shouldSaveFrames = true;
//    }
//
//    /**
//     * Initialize the folders and add another one each time this runs,
//     *   incrementing the name by one to keep the runs separate
//     */
//    @SuppressWarnings("ResultOfMethodCallIgnored")
//    private void initFolder() {
//        File pictures = new File("/storage/self/primary/Pictures/webcam");         // the Pictures/webcam folder on the RC phone (/storage/self/primary/Pictures/webcam)
//
//        // Get an array of the old folders we've made so we can find the last number we've used.
//        File[] oldFolders = pictures.listFiles(new FilenameFilter() { // filter by the filename
//            @Override
//            public boolean accept(File folder, String name) {
//                return name.startsWith("webcamManager"); // if the name starts with webcamManager, it's probably ours.
//            }
//        });
//
//        // find the last number we used
//        int maxFolderIndex = -1; // -1 since we're adding 1 later and want to start at 0 if we find no old folders
//        if(oldFolders != null)
//            for (File folder : oldFolders) // loop through all the folders that matched our search
//                if (Integer.parseInt(folder.getName().split("_")[1]) > maxFolderIndex) // parse the int following the hyphen and compare with the maxFolderIndex
//                    maxFolderIndex = Integer.parseInt(folder.getName().split("_")[1]); // If it's bigger than the max, save it
//
//        // the current folder we're working in; Pictures/webcam/webcamManager_#
//        folder = new File(pictures, "webcamManager_" + (maxFolderIndex + 1));
//        originals = new File(folder, "originals"); // the originals subfolder
//        userSupplied = new File(folder, "user");   // the user subfolder
//
//        // now we need to make these fictional folders if they don't exist...
//        pictures.mkdir();
//        folder.mkdir();
//        originals.mkdir();
//        userSupplied.mkdir();
//    }
//
//
//    /**
//     * Internal. Save the bitmap to a specified folder (originals or userSupplied) using the counter specified
//     * @param folder  the folder to save to
//     * @param counter the counter for the filename
//     * @param bitmap  the bitmap to save
//     */
//    private void saveBitmapInternal(File folder, int counter, Bitmap bitmap) {
//        // make sure we should be doing this...
//        if (shouldSaveFrames)
//            try {
//                // Create an output stream to the png file in the folder using the counter for the filename
//                FileOutputStream outStream = new FileOutputStream(new File(folder, counter + ".png"));
//
//                // Compress the bitmap into a PNG image to the FileOutputStream outStream
//                bitmap.compress(Bitmap.CompressFormat.PNG, 100, outStream);
//
//                outStream.close(); // Close the stream.
//
//            } catch (Exception e) {
//                RobotLog.d("Error saving bitmap!");
//                RobotLog.d(e.getMessage());
//            }
//    }
//
//    /**
//     * Save a frame direct from the webcam
//     * @param frame the frame to save
//     */
//    private void saveFrame(Bitmap frame) {
//        // increment and check if we should save this frame
//        // (framesToSkip + 1 % framesSaved should give us 0 if we should save this one
//        // eg. framesToSkip is 5, we'll save 0, 6, 12, 18, skipping 5 each time.
//        RobotLog.d("FRAMES SAVED:" + framesSaved);
//        if ((framesSaved++) % (framesToSkip + 1) == 0) {
//            // Save the frame to the originals folder using the counter framesSaved
//            saveBitmapInternal(originals, framesSaved, frame);
//            RobotLog.d("FRAME SAVED SUCCESSFUL!");
//        }
//    }
//
//
//    /**
//     * Save a publicly-supplied bitmap
//     * @param bitmap the bitmap to save
//     */
//    public void saveBitmap(Bitmap bitmap) {
//        // Save it to the userSupplied folder using and incrementing the userFramesSaved file counter.
//        saveBitmapInternal(userSupplied, userFramesSaved++, bitmap);
//    }
//
//
//
//    public static void colorToHSV(int color, double[]hsv) {
//        double r = Color.red(color) / 255d;
//        double b = Color.blue(color) / 255d;
//        double g = Color.green(color) / 255d;
//        double cmin = Math.min(r, Math.min(b, g));
//        double cmax = Math.max(r, Math.max(b, g));
//        double delta = cmax - cmin;
//
//        hsv[0] = delta == 0 ? 0 :
//                cmax == r ? 60 * ((g-b)/delta) :
//                        cmax == g ? 60 * ((b-r)/delta + 2) :
//                                60 * ((r-g)/delta + 4);
//        hsv[1] = cmax == 0 ? 0 : delta / cmax;
//        hsv[2] = cmax;
//    }
//}