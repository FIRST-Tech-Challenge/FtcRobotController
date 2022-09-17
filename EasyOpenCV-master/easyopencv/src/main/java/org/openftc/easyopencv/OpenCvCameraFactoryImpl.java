/*
 * Copyright (c) 2019 OpenFTC Team
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
import android.content.pm.PackageManager;
import android.view.View;
import android.view.ViewGroup;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.ArrayList;
import java.util.concurrent.CountDownLatch;

class OpenCvCameraFactoryImpl extends OpenCvCameraFactory
{
    private static int appVersion = -1;
    private static int sdk6_1_versionCode = 39;
    private static String sdk_6_1 = "6.1";

    static
    {
        try
        {
            appVersion = AppUtil.getInstance().getApplication().getPackageManager().getPackageInfo(AppUtil.getInstance().getApplication().getPackageName(), 0).versionCode;
        }
        catch (PackageManager.NameNotFoundException e)
        {
            e.printStackTrace();
        }
    }

    private void throwIfIncompatibleSdkVersion()
    {
        if(appVersion < sdk6_1_versionCode)
        {
            throw new RuntimeException(String.format("EasyOpenCV v%s is only compatible with SDK v%s or greater!", BuildConfig.VERSION_NAME, sdk_6_1));
        }
    }

    @Override
    public OpenCvInternalCamera createInternalCamera(OpenCvInternalCamera.CameraDirection direction)
    {
        throwIfIncompatibleSdkVersion();
        return new OpenCvInternalCameraImpl(direction);
    }

    @Override
    public OpenCvInternalCamera createInternalCamera(OpenCvInternalCamera.CameraDirection direction, int containerId)
    {
        throwIfIncompatibleSdkVersion();
        return new OpenCvInternalCameraImpl(direction, containerId);
    }

    @Override
    public OpenCvInternalCamera2 createInternalCamera2(OpenCvInternalCamera2.CameraDirection direction)
    {
        throwIfIncompatibleSdkVersion();
        return new OpenCvInternalCamera2Impl(direction);
    }

    @Override
    public OpenCvInternalCamera2 createInternalCamera2(OpenCvInternalCamera2.CameraDirection direction, int containerId)
    {
        throwIfIncompatibleSdkVersion();
        return new OpenCvInternalCamera2Impl(direction, containerId);
    }

    @Override
    public OpenCvWebcam createWebcam(WebcamName webcamName)
    {
        throwIfIncompatibleSdkVersion();
        return new OpenCvWebcamImpl(webcamName);
    }

    @Override
    public OpenCvWebcam createWebcam(WebcamName webcamName, int viewportContainerId)
    {
        throwIfIncompatibleSdkVersion();
        return new OpenCvWebcamImpl(webcamName, viewportContainerId);
    }

    @Override
    public OpenCvSwitchableWebcam createSwitchableWebcam(WebcamName... names)
    {
        throwIfIncompatibleSdkVersion();
        SwitchableCameraName cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(names);

        return new OpenCvSwitchableWebcamImpl(cameraName);
    }

    @Override
    public OpenCvSwitchableWebcam createSwitchableWebcam(int viewportContainerId, WebcamName... names)
    {
        throwIfIncompatibleSdkVersion();
        SwitchableCameraName cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(names);

        return new OpenCvSwitchableWebcamImpl(cameraName, viewportContainerId);
    }

    @Override
    public OpenCvCamera createVuforiaPassthrough(VuforiaLocalizer vuforiaLocalizer, VuforiaLocalizer.Parameters parameters, int viewportId)
    {
        return new OpenCvVuforiaPassthroughImpl(vuforiaLocalizer, parameters, viewportId);
    }

    @Override
    public OpenCvCamera createVuforiaPassthrough(VuforiaLocalizer vuforiaLocalizer, VuforiaLocalizer.Parameters parameters)
    {
        return new OpenCvVuforiaPassthroughImpl(vuforiaLocalizer, parameters);
    }

    @Override
    public int[] splitLayoutForMultipleViewports(final int containerId, final int numViewports, final ViewportSplitMethod viewportSplitMethod)
    {
        throwIfIncompatibleSdkVersion();

        if(numViewports < 2)
        {
            throw new IllegalArgumentException("Layout requested to be split for <2 viewports!");
        }

        final int[] ids = new int[numViewports];
        final ArrayList<LinearLayout> layoutArrayList = new ArrayList<>(numViewports);

        final CountDownLatch latch = new CountDownLatch(1);

        //We do the viewport creation on the UI thread, but if there's an exception then
        //we need to catch it and rethrow it on the OpMode thread
        final RuntimeException[] exToRethrowOnOpModeThread = {null};

        AppUtil.getInstance().getActivity().runOnUiThread(new Runnable()
        {
            @Override
            public void run()
            {
                try
                {
                    final LinearLayout containerLayout = (LinearLayout) AppUtil.getInstance().getActivity().findViewById(containerId);

                    if(containerLayout == null)
                    {
                        throw new OpenCvCameraException("Viewport container specified by user does not exist!");
                    }
                    else if(containerLayout.getChildCount() != 0)
                    {
                        throw new OpenCvCameraException("Viewport container specified by user is not empty!");
                    }

                    containerLayout.setVisibility(View.VISIBLE);

                    if(viewportSplitMethod == null)
                    {
                        throw new IllegalArgumentException("Viewport split method cannot be null!");
                    }
                    else if(viewportSplitMethod == ViewportSplitMethod.VERTICALLY)
                    {
                        containerLayout.setOrientation(LinearLayout.VERTICAL);
                    }
                    else
                    {
                        containerLayout.setOrientation(LinearLayout.HORIZONTAL);
                    }

                    for(int i = 0; i < numViewports; i++)
                    {
                        LinearLayout linearLayout = new LinearLayout(AppUtil.getInstance().getActivity());
                        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT);
                        params.weight = 1;
                        linearLayout.setLayoutParams(params);
                        linearLayout.setId(View.generateViewId());
                        ids[i] = linearLayout.getId();
                        layoutArrayList.add(linearLayout);
                        containerLayout.addView(linearLayout);
                    }

                    LIFO_OpModeCallbackDelegate.getInstance().add(new LIFO_OpModeCallbackDelegate.OnOpModeStoppedListener()
                    {
                        @Override
                        public void onOpModePostStop(OpMode opMode)
                        {
                            AppUtil.getInstance().getActivity().runOnUiThread(new Runnable()
                            {
                                @Override
                                public void run()
                                {
                                    for(LinearLayout layout : layoutArrayList)
                                    {
                                        containerLayout.removeView(layout);
                                    }
                                    containerLayout.setVisibility(View.GONE);
                                    containerLayout.setOrientation(LinearLayout.VERTICAL);
                                }
                            });

                        }
                    });

                    latch.countDown();
                }
                catch (RuntimeException e)
                {
                    exToRethrowOnOpModeThread[0] = e;
                }

            }
        });

        if(exToRethrowOnOpModeThread[0] != null)
        {
            throw exToRethrowOnOpModeThread[0];
        }

        try
        {
            latch.await();

            return ids;
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
            return null;
        }
    }
}
