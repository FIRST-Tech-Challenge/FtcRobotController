/*
 * Copyright (c) 2020 OpenFTC Team
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

import android.media.MediaCodec;
import android.media.MediaRecorder;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.TimeZone;

public class PipelineRecordingParameters
{
    public final String path;
    public final Encoder encoder;
    public final OutputFormat outputFormat;
    public final int bitrate;
    public final int frameRate;

    public enum Encoder
    {
        H264(MediaRecorder.VideoEncoder.H264),
        H263(MediaRecorder.VideoEncoder.H263),
        VP8(MediaRecorder.VideoEncoder.VP8),
        MPEG_4_SP(MediaRecorder.VideoEncoder.MPEG_4_SP);

        final int format;

        Encoder(int format)
        {
            this.format = format;
        }
    }

    public enum OutputFormat
    {
        MPEG_4(MediaRecorder.OutputFormat.MPEG_4),
        THREE_GPP(MediaRecorder.OutputFormat.THREE_GPP),
        WEBM(MediaRecorder.OutputFormat.WEBM);

        final int format;

        OutputFormat(int format)
        {
            this.format = format;
        }
    }

    public enum BitrateUnits
    {
        bps(1),
        Kbps(1000),
        Mbps(1000000);

        final int scalar;

        BitrateUnits(int scalar)
        {
            this.scalar = scalar;
        }
    }

    public PipelineRecordingParameters(OutputFormat outputFormat, Encoder encoder, int frameRate, int bitrate, String path)
    {
        this.outputFormat = outputFormat;
        this.encoder = encoder;
        this.frameRate = frameRate;
        this.bitrate = bitrate;
        this.path = path;
    }

    public static class Builder
    {
        private String path = "/sdcard/EasyOpenCV/pipeline_recording_"+new SimpleDateFormat("dd-MM-yyyy_HH:mm:ss", Locale.getDefault()).format(new Date())+".mp4";
        private Encoder encoder = Encoder.H264;
        private OutputFormat outputFormat = OutputFormat.MPEG_4;
        private int bitrate = 4000000;
        private int frameRate = 30;

        public Builder setPath(String path)
        {
            this.path = path;
            return this;
        }

        public Builder setEncoder(Encoder encoder)
        {
            this.encoder = encoder;
            return this;
        }

        public Builder setOutputFormat(OutputFormat outputFormat)
        {
            this.outputFormat = outputFormat;
            return this;
        }

        public Builder setBitrate(int bitrate, BitrateUnits units)
        {
            this.bitrate = bitrate*units.scalar;
            return this;
        }

        public Builder setFrameRate(int frameRate)
        {
            this.frameRate = frameRate;
            return this;
        }

        public PipelineRecordingParameters build()
        {
            return new PipelineRecordingParameters(outputFormat, encoder, frameRate, bitrate, path);
        }
    }
}
