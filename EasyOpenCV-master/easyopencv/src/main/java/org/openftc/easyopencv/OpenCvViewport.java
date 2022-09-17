/*
 * Copyright (c) 2018 OpenFTC Team
 *
 * Note: credit where credit is due - some parts of OpenCv's
 *       JavaCameraView were used as a reference
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.openftc.easyopencv;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

public class OpenCvViewport extends SurfaceView implements SurfaceHolder.Callback
{
    private Size size;
    private Bitmap bitmapFromMat;
    private RenderThread renderThread;
    private Canvas canvas = null;
    private double aspectRatio;
    private static final int VISION_PREVIEW_FRAME_QUEUE_CAPACITY = 2;
    private static final int FRAMEBUFFER_RECYCLER_CAPACITY = VISION_PREVIEW_FRAME_QUEUE_CAPACITY + 2; //So that the evicting queue can be full, and the render thread has one checked out (+1) and post() can still take one (+1).
    private EvictingBlockingQueue<MatRecycler.RecyclableMat> visionPreviewFrameQueue = new EvictingBlockingQueue<>(new ArrayBlockingQueue<MatRecycler.RecyclableMat>(VISION_PREVIEW_FRAME_QUEUE_CAPACITY));
    private MatRecycler framebufferRecycler;
    private volatile RenderingState internalRenderingState = RenderingState.STOPPED;
    private int statBoxW = 450;
    private int statBoxH = 120;
    private int statBoxTextLineSpacing = 35;
    private int statBoxLTxtMargin = 5;
    private final Object syncObj = new Object();
    private volatile boolean userRequestedActive = false;
    private volatile boolean userRequestedPause = false;
    private boolean needToDeactivateRegardlessOfUser = false;
    private boolean surfaceExistsAndIsReady = false;
    private Paint fpsMeterNormalBgPaint;
    private Paint fpsMeterRecordingPaint;
    private Paint fpsMeterTextPaint;
    private Paint paintBlackBackground;
    private boolean fpsMeterEnabled = true;
    private float fps = 0;
    private int pipelineMs = 0;
    private int overheadMs = 0;
    private String TAG = "OpenCvViewport";
    private ReentrantLock renderThreadAliveLock = new ReentrantLock();
    private volatile OptimizedRotation optimizedViewRotation;
    private volatile OpenCvCamera.ViewportRenderer renderer = OpenCvCamera.ViewportRenderer.SOFTWARE;
    private volatile OpenCvCamera.ViewportRenderingPolicy renderingPolicy = OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY;
    private volatile boolean isRecording;

    public OpenCvViewport(Context context, OnClickListener onClickListener)
    {
        super(context);

        fpsMeterNormalBgPaint = new Paint();
        fpsMeterNormalBgPaint.setColor(Color.rgb(102, 20, 68));
        fpsMeterNormalBgPaint.setStyle(Paint.Style.FILL);

        fpsMeterRecordingPaint = new Paint();
        fpsMeterRecordingPaint.setColor(Color.rgb(255, 0, 0));
        fpsMeterRecordingPaint.setStyle(Paint.Style.FILL);

        fpsMeterTextPaint = new Paint();
        fpsMeterTextPaint.setColor(Color.WHITE);
        fpsMeterTextPaint.setTextSize(30);

        paintBlackBackground = new Paint();
        paintBlackBackground.setColor(Color.BLACK);
        paintBlackBackground.setStyle(Paint.Style.FILL);

        getHolder().addCallback(this);

        visionPreviewFrameQueue.setEvictAction(new Consumer<MatRecycler.RecyclableMat>()
        {
            @Override
            public void accept(MatRecycler.RecyclableMat value)
            {
                /*
                 * If a Mat is evicted from the queue, we need
                 * to make sure to return it to the Mat recycler
                 */
                framebufferRecycler.returnMat(value);
            }
        });

        setOnClickListener(onClickListener);
    }

    private enum RenderingState
    {
        STOPPED,
        ACTIVE,
        PAUSED,
    }

    public enum OptimizedRotation
    {
        NONE(0),
        ROT_90_COUNTERCLOCWISE(90),
        ROT_90_CLOCKWISE(-90),
        ROT_180(180);

        int val;

        OptimizedRotation(int val)
        {
            this.val = val;
        }
    }

    public void setRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy policy)
    {
        renderingPolicy = policy;
    }

    public void setRenderer(OpenCvCamera.ViewportRenderer renderer) throws IllegalStateException
    {
        synchronized (syncObj)
        {
            if(internalRenderingState != RenderingState.STOPPED)
            {
                throw new IllegalStateException();
            }
            else
            {
                this.renderer = renderer;
            }
        }
    }

    public void setRecording(boolean recording)
    {
        synchronized (syncObj)
        {
            isRecording = recording;
        }
    }

    public void setSize(Size size)
    {
        synchronized (syncObj)
        {
            if(internalRenderingState != RenderingState.STOPPED)
            {
                throw new IllegalStateException("Cannot set size while renderer is active!");
            }

            //did they give us null?
            if(size == null)
            {
                //ugh, they did
                throw new IllegalArgumentException("size cannot be null!");
            }

            this.size = size;
            this.aspectRatio = (double)size.getWidth() / (double)size.getHeight();

            //Make sure we don't have any mats hanging around
            //from when we might have been running before
            visionPreviewFrameQueue.clear();

            framebufferRecycler = new MatRecycler(FRAMEBUFFER_RECYCLER_CAPACITY);
        }
    }

    public void setOptimizedViewRotation(OptimizedRotation optimizedViewRotation)
    {
        this.optimizedViewRotation = optimizedViewRotation;
    }

    public void post(Mat mat)
    {
        synchronized (syncObj)
        {
            //did they give us null?
            if(mat == null)
            {
                //ugh, they did
                throw new IllegalArgumentException("cannot post null mat!");
            }

            //Are we actually rendering to the display right now? If not,
            //no need to waste time doing a memcpy
            if(internalRenderingState == RenderingState.ACTIVE)
            {
                /*
                 * We need to copy this mat before adding it to the queue,
                 * because the pointer that was passed in here is only known
                 * to be pointing to a certain frame while we're executing.
                 */
                try
                {
                    /*
                     * Grab a framebuffer Mat from the recycler
                     * instead of doing a new alloc and then having
                     * to free it after rendering/eviction from queue
                     */
                    MatRecycler.RecyclableMat matToCopyTo = framebufferRecycler.takeMat();
                    mat.copyTo(matToCopyTo);
                    visionPreviewFrameQueue.offer(matToCopyTo);
                }
                catch (InterruptedException e)
                {
                    e.printStackTrace();
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    /*
     * Called with syncObj held
     */
    public void checkState()
    {
        /*
         * If the surface isn't ready, don't do anything
         */
        if(!surfaceExistsAndIsReady)
        {
            Log.d(TAG, "CheckState(): surface not ready or doesn't exist");
            return;
        }

        /*
         * Does the user want us to stop?
         */
        if(!userRequestedActive || needToDeactivateRegardlessOfUser)
        {
            if(needToDeactivateRegardlessOfUser)
            {
                Log.d(TAG, "CheckState(): lifecycle mandates deactivation regardless of user");
            }
            else
            {
                Log.d(TAG, "CheckState(): user requested that we deactivate");
            }

            /*
             * We only need to stop the render thread if it's not
             * already stopped
             */
            if(internalRenderingState != RenderingState.STOPPED)
            {
                Log.d(TAG, "CheckState(): deactivating viewport");

                /*
                 * Interrupt him so he's not stuck looking at his
                 * frame queue.
                 */
                renderThread.notifyExitRequested();
                renderThread.interrupt();

                /*
                 * Wait for him to die non-interuptibly
                 */
                renderThreadAliveLock.lock();
                renderThreadAliveLock.unlock();

//                try
//                {
//                    /*
//                     * Wait for him to die
//                     */
//                    renderThread.join();
//                }
//                catch (InterruptedException e)
//                {
//                    e.printStackTrace();
//                }

                internalRenderingState = RenderingState.STOPPED;
            }
            else
            {
                Log.d(TAG, "CheckState(): already deactivated");
            }
        }

        /*
         * Does the user want us to start?
         */
        else if(userRequestedActive)
        {
            Log.d(TAG, "CheckState(): user requested that we activate");

            /*
             * We only need to start the render thread if it's
             * stopped.
             */
            if(internalRenderingState == RenderingState.STOPPED)
            {
                Log.d(TAG, "CheckState(): activating viewport");

                internalRenderingState = RenderingState.PAUSED;

                if(userRequestedPause)
                {
                    internalRenderingState = RenderingState.PAUSED;
                }
                else
                {
                    internalRenderingState = RenderingState.ACTIVE;
                }

                renderThread = new RenderThread();
                renderThread.start();
            }
            else
            {
                Log.d(TAG, "CheckState(): already activated");
            }
        }

        if(internalRenderingState != RenderingState.STOPPED)
        {
            if(userRequestedPause && internalRenderingState != RenderingState.PAUSED
                    || !userRequestedPause && internalRenderingState != RenderingState.ACTIVE)
            {
                if(userRequestedPause)
                {
                    Log.d(TAG, "CheckState(): pausing viewport");
                    internalRenderingState = RenderingState.PAUSED;
                }
                else
                {
                    Log.d(TAG, "CheckState(): resuming viewport");
                    internalRenderingState = RenderingState.ACTIVE;
                }

                /*
                 * Interrupt him so that he's not stuck looking at his frame queue.
                 * (We stop filling the frame queue if the user requested pause so
                 * we aren't doing pointless memcpys)
                 */
                renderThread.interrupt();
            }
        }
    }

    /***
     * Activate the render thread
     */
    public synchronized void activate()
    {
        synchronized (syncObj)
        {
            userRequestedActive = true;
            checkState();
        }
    }

    /***
     * Deactivate the render thread
     */
    public void deactivate()
    {
        synchronized (syncObj)
        {
            userRequestedActive = false;
            checkState();
        }
    }

    public void resume()
    {
        synchronized (syncObj)
        {
            userRequestedPause = false;
            checkState();
        }
    }

    public void pause()
    {
        synchronized (syncObj)
        {
            userRequestedPause = true;
            checkState();
        }
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder)
    {
        Log.d(TAG, "surfaceCreated()");
        Log.d(TAG, "...surfaceCreated()");
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height)
    {
        Log.d(TAG, "surfaceChanged()");

        synchronized (syncObj)
        {
            needToDeactivateRegardlessOfUser = false;
            surfaceExistsAndIsReady = true;

            checkState();
        }

        Log.d(TAG, "...surfaceChanged()");
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder)
    {
        /*
         * NOTE: The docs for this method speak thusly:
         *     if you have a rendering thread that directly accesses the surface,
         *     you must ensure that thread is no longer touching the Surface before
         *     returning from this function.
         *
         * We handle this by waiting UNINTERRUPTIBLY for the render thread to exit
         * in checkState().
         */

        Log.d(TAG, "surfaceDestroyed()");

        synchronized (syncObj)
        {
            needToDeactivateRegardlessOfUser = true;
            checkState();
            surfaceExistsAndIsReady = false;
        }

        Log.d(TAG, "...surfaceDestroyed()");
    }

    public void setFpsMeterEnabled(boolean fpsMeterEnabled)
    {
        this.fpsMeterEnabled = fpsMeterEnabled;
    }

    public void notifyStatistics(float fps, int pipelineMs, int overheadMs)
    {
        this.fps = fps;
        this.pipelineMs = pipelineMs;
        this.overheadMs = overheadMs;
    }

    class RenderThread extends Thread
    {
        boolean shouldPaintOrange = true;
        volatile boolean exitRequested = false;
        private String TAG = "OpenCvViewportRenderThread";

        public void notifyExitRequested()
        {
            exitRequested = true;
        }

        private Canvas lockCanvas()
        {
            if(renderer == OpenCvCamera.ViewportRenderer.SOFTWARE)
            {
                return getHolder().lockCanvas();
            }
            else if(renderer == OpenCvCamera.ViewportRenderer.GPU_ACCELERATED)
            {
                return getHolder().getSurface().lockHardwareCanvas();
            }

            throw new IllegalStateException();
        }

        private void swapBuffer(Canvas canvas)
        {
            if(renderer == OpenCvCamera.ViewportRenderer.SOFTWARE)
            {
                getHolder().unlockCanvasAndPost(canvas);
            }
            else if(renderer == OpenCvCamera.ViewportRenderer.GPU_ACCELERATED)
            {
                getHolder().getSurface().unlockCanvasAndPost(canvas);
            }
            else
            {
                throw new IllegalStateException();
            }
        }

        @Override
        public void run()
        {
            renderThreadAliveLock.lock();

            //Make sure we don't have any mats hanging around
            //from when we might have been running before
            visionPreviewFrameQueue.clear();

            Log.d(TAG, "Render thread is up!");

            bitmapFromMat = Bitmap.createBitmap(size.getWidth(), size.getHeight(), Bitmap.Config.ARGB_8888);

            canvas = lockCanvas();
            canvas.drawColor(Color.BLUE);
            swapBuffer(canvas);

            while (!exitRequested)
            {
                switch (internalRenderingState)
                {
                    case ACTIVE:
                    {
                        shouldPaintOrange = true;

                        MatRecycler.RecyclableMat mat;

                        try
                        {
                            //Grab a Mat from the frame queue
                            mat = visionPreviewFrameQueue.take();
                        }
                        catch (InterruptedException e)
                        {
                            e.printStackTrace();
                            //Note: we actually don't re-interrupt ourselves here, because interrupts are also
                            //used to simply make sure we properly pick up a transition to the PAUSED state, not
                            //just when we're trying to close. If we're trying to close, then exitRequested will
                            //be set, and since we break immediately right here, the close will be handled cleanly.
                            //Thread.currentThread().interrupt();
                            break;
                        }

                        //Get canvas object for rendering on
                        canvas = lockCanvas();

                        /*
                         * For some reason, the canvas will very occasionally be null upon closing.
                         * Stack Overflow seems to suggest this means the canvas has been destroyed.
                         * However, surfaceDestroyed(), which is called right before the surface is
                         * destroyed, calls checkState(), which *SHOULD* block until we die. This
                         * works most of the time, but not always? We don't yet understand...
                         */
                        if(canvas != null)
                        {
                            //Convert that Mat to a bitmap we can render
                            Utils.matToBitmap(mat, bitmapFromMat);

                            //Draw the background each time to prevent double buffering problems
                            canvas.drawColor(Color.rgb(239,239,239)); // RC activity background color

                            if(renderingPolicy == OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY)
                            {
                                drawOptimizingEfficiency(canvas);
                            }
                            else if(renderingPolicy == OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW)
                            {
                                drawOptimizingView(canvas);
                            }

                            swapBuffer(canvas);
                        }
                        else
                        {
                            Log.d(TAG, "Canvas was null");
                        }

                        //We're done with that Mat object; return it to the Mat recycler so it can be used again later
                        framebufferRecycler.returnMat(mat);

                        break;
                    }

                    case PAUSED:
                    {
                        if(shouldPaintOrange)
                        {
                            shouldPaintOrange = false;

                            canvas = lockCanvas();

                            /*
                             * For some reason, the canvas will very occasionally be null upon closing.
                             * Stack Overflow seems to suggest this means the canvas has been destroyed.
                             * However, surfaceDestroyed(), which is called right before the surface is
                             * destroyed, calls checkState(), which *SHOULD* block until we die. This
                             * works most of the time, but not always? We don't yet understand...
                             */
                            if(canvas != null)
                            {
                                canvas.drawColor(Color.rgb(255, 166, 0));
                                canvas.drawRect(0, canvas.getHeight()-40, 450, canvas.getHeight(), fpsMeterNormalBgPaint);
                                canvas.drawText("VIEWPORT PAUSED", 5, canvas.getHeight()-10, fpsMeterTextPaint);
                                swapBuffer(canvas);
                            }
                        }

                        try
                        {
                            Thread.sleep(50);
                        }
                        catch (InterruptedException e)
                        {
                            e.printStackTrace();
                            //Note: we actually don't re-interrupt ourselves here, because interrupts are also
                            //used to simply make sure we properly pick up a transition to the PAUSED state, not
                            //just when we're trying to close. If we're trying to close, then exitRequested will
                            //be set, and since we break immediately right here, the close will be handled cleanly.
                            //Thread.currentThread().interrupt();
                        }
                        break;
                    }
                }
            }

            Log.d(TAG, "About to exit");
            bitmapFromMat.recycle(); //Help the garbage collector :)
            renderThreadAliveLock.unlock();
        }

        void drawOptimizingView(Canvas canvas)
        {
            /***
             * WE CAN ONLY LOOK AT THIS VARIABLE ONCE BECAUSE IT CAN BE CHANGED BEHIND
             * OUT BACKS FROM ANOTHER THREAD!
             *
             * Technically, we could synchronize with {@link #setOptimizedViewRotation(OptimizedRotation)}
             * but drawing can sometimes take a long time (e.g. 30ms) so just caching seems to be better...
             */
            OptimizedRotation optimizedViewRotationLocalCache = optimizedViewRotation;

            if(optimizedViewRotationLocalCache == OptimizedRotation.NONE)
            {
                /*
                 * Ignore this request to optimize the view, nothing to do
                 */
                drawOptimizingEfficiency(canvas);
                return;
            }
            else if(optimizedViewRotationLocalCache == OptimizedRotation.ROT_180)
            {
                /*
                 * If we're rotating by 180, then we can just re-use the drawing code
                 * from the efficient method
                 */
                canvas.rotate(optimizedViewRotationLocalCache.val, canvas.getWidth()/2, canvas.getHeight()/2);
                drawOptimizingEfficiency(canvas);
                return;
            }

            drawOptimizingViewForQuarterRot(canvas, optimizedViewRotationLocalCache);
        }

        void drawOptimizingViewForQuarterRot(Canvas canvas, OptimizedRotation optimizedViewRotationLocalCache)
        {
            canvas.rotate(optimizedViewRotationLocalCache.val, canvas.getWidth()/2, canvas.getHeight()/2);

            // Swapped because of 90deg rotation
            int canvasWidth = canvas.getHeight();
            int canvasHeight = canvas.getWidth();

            // Calculate the new origin (top left) that will be visible onscreen
            int origin_x = (canvasHeight-canvasWidth)/2;
            int origin_y = (canvasWidth-canvasHeight)/2;

            // Calculate the aspect ratio of the canvas
            double canvasAspect = (float) canvasWidth/canvasHeight;

            int y_offset_statbox = 0;

            // Image width is the factor we need to consider
            if(aspectRatio > canvasAspect)
            {
                // Width: we use the max we have, since horizontal bounds are hit before vertical bounds
                int scaledWidth = canvasWidth;

                // Height: calculate a scaled height assuming width is maxed for the canvas
                int scaledHeight = (int) Math.round(canvasWidth / aspectRatio);

                // We want to center the image in the viewport
                int topLeftX = origin_x;
                int topLeftY = origin_y + Math.abs(canvasHeight-scaledHeight)/2;
                y_offset_statbox = Math.abs(canvasHeight-scaledHeight)/2;

                Rect rect = createRect(
                                topLeftX,
                                topLeftY,
                                scaledWidth,
                                scaledHeight);

                // Draw black behind the bitmap to avoid alpha issues if usercode tries to draw
                // annotations and doesn't specify alpha 255. This wasn't an issue when we just
                // painted black behind the entire view, but now that we paint the RC background
                // color, it is an issue...
                canvas.drawRect(rect, paintBlackBackground);

                canvas.drawBitmap(
                        bitmapFromMat,
                        null,
                        rect,
                        null
                );
            }
            // Image height is the factor we need to consider
            else
            {
                // Height: we use the max we have, since vertical bounds are hit before horizontal bounds
                int scaledHeight = canvasHeight;

                // Width: calculate a scaled width assuming height is maxed for the canvas
                int scaledWidth = (int) Math.round(canvasHeight * aspectRatio);

                // We want to center the image in the viewport
                int topLeftY = origin_y;
                int topLeftX = origin_x + Math.abs(canvasWidth-scaledWidth)/2;

                Rect rect = createRect(
                        topLeftX,
                        topLeftY,
                        scaledWidth,
                        scaledHeight);

                // Draw black behind the bitmap to avoid alpha issues if usercode tries to draw
                // annotations and doesn't specify alpha 255. This wasn't an issue when we just
                // painted black behind the entire view, but now that we paint the RC background
                // color, it is an issue...
                canvas.drawRect(rect, paintBlackBackground);

                canvas.drawBitmap(
                        bitmapFromMat,
                        null,
                        rect,
                        null
                );
            }

            /*
             * If we don't need to draw the statistics, get out of dodge
             */
            if(!fpsMeterEnabled)
                return;

            Rect rect = null;

            if(optimizedViewRotationLocalCache == OptimizedRotation.ROT_90_COUNTERCLOCWISE)
            {
                rect = createRect(
                        origin_x+canvasWidth-statBoxW,
                        origin_y+canvasHeight-statBoxH-y_offset_statbox,
                        statBoxW,
                        statBoxH);
            }
            else if(optimizedViewRotationLocalCache == OptimizedRotation.ROT_90_CLOCKWISE)
            {
                rect = createRect(
                        origin_x,
                        origin_y+canvasHeight-statBoxH-y_offset_statbox,
                        statBoxW,
                        statBoxH);
            }

            drawStats(canvas, rect);
        }

        void drawOptimizingEfficiency(Canvas canvas)
        {
            int x_offset_statbox = 0;
            int y_offset_statbox = 0;

            /*
             * We need to draw minding the HEIGHT we have to work with; width is not an issue
             */
            if((canvas.getHeight() * aspectRatio) < canvas.getWidth())
            {
                // Height: we use the max we have, since vertical bounds are hit before horizontal bounds
                int scaledHeight = canvas.getHeight();

                // Width: calculate a scaled width assuming height is maxed for the canvas
                int scaledWidth = (int) Math.round(canvas.getHeight() * aspectRatio);

                // We want to center the image in the viewport
                x_offset_statbox = Math.abs(canvas.getWidth()-scaledWidth)/2;
                int topLeftY = 0;
                int topLeftX = 0 + Math.abs(canvas.getWidth()-scaledWidth)/2;

                //Draw the bitmap, scaling it to the maximum size that will fit in the viewport
                Rect rect = createRect(
                        topLeftX,
                        topLeftY,
                        scaledWidth,
                        scaledHeight);

                // Draw black behind the bitmap to avoid alpha issues if usercode tries to draw
                // annotations and doesn't specify alpha 255. This wasn't an issue when we just
                // painted black behind the entire view, but now that we paint the RC background
                // color, it is an issue...
                canvas.drawRect(rect, paintBlackBackground);

                canvas.drawBitmap(
                        bitmapFromMat,
                        null,
                        rect,
                        null
                );
            }

            /*
             * We need to draw minding the WIDTH we have to work with; height is not an issue
             */
            else
            {
                // Width: we use the max we have, since horizontal bounds are hit before vertical bounds
                int scaledWidth = canvas.getWidth();

                // Height: calculate a scaled height assuming width is maxed for the canvas
                int scaledHeight = (int) Math.round(canvas.getWidth() / aspectRatio);

                // We want to center the image in the viewport
                int topLeftY = Math.abs(canvas.getHeight()-scaledHeight)/2;
                int topLeftX = 0;
                y_offset_statbox = Math.abs(canvas.getHeight()-scaledHeight)/2;

                //Draw the bitmap, scaling it to the maximum size that will fit in the viewport
                Rect rect = createRect(
                        topLeftX,
                        topLeftY,
                        scaledWidth,
                        scaledHeight);

                // Draw black behind the bitmap to avoid alpha issues if usercode tries to draw
                // annotations and doesn't specify alpha 255. This wasn't an issue when we just
                // painted black behind the entire view, but now that we paint the RC background
                // color, it is an issue...
                canvas.drawRect(rect, paintBlackBackground);

                canvas.drawBitmap(
                        bitmapFromMat,
                        null,
                        rect,
                        null
                );
            }

            /*
             * If we don't need to draw the statistics, get out of dodge
             */
            if(!fpsMeterEnabled)
                return;

            Rect rect = createRect(
                    x_offset_statbox,
                    canvas.getHeight()-statBoxH-y_offset_statbox,
                    statBoxW,
                    statBoxH
            );

            drawStats(canvas, rect);
        }
    }

    void drawStats(Canvas canvas, Rect rect)
    {
        // Draw the purple rectangle
        if(isRecording)
        {
            canvas.drawRect(rect, fpsMeterRecordingPaint);
        }
        else
        {
            canvas.drawRect(rect, fpsMeterNormalBgPaint);
        }

        // Some formatting stuff
        int statBoxLTxtStart = rect.left+statBoxLTxtMargin;
        int textLine1Y = rect.bottom - 80;
        int textLine2Y = textLine1Y + statBoxTextLineSpacing;
        int textLine3Y = textLine2Y + statBoxTextLineSpacing;

        // Draw the 3 text lines
        canvas.drawText(String.format("OpenFTC EasyOpenCV v%s", BuildConfig.VERSION_NAME),        statBoxLTxtStart, textLine1Y, fpsMeterTextPaint);
        canvas.drawText(String.format("FPS@%dx%d: %.2f", size.getWidth(), size.getHeight(), fps), statBoxLTxtStart, textLine2Y, fpsMeterTextPaint);
        canvas.drawText(String.format("Pipeline: %dms - Overhead: %dms", pipelineMs, overheadMs), statBoxLTxtStart, textLine3Y, fpsMeterTextPaint);
    }

    Rect createRect(int tlx, int tly, int w, int h)
    {
        return new Rect(tlx, tly, tlx+w, tly+h);
    }
}