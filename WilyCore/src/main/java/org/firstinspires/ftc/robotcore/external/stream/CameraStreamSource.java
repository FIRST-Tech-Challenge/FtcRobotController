/*
Copyright (c) 2019 Ryan Brott

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Ryan Brott nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcore.external.stream;


import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;

/**
 * Interface representing an on-demand source of frames (i.e., bitmaps).
 */
public interface CameraStreamSource {

    /**
     * Requests a single frame bitmap.
     *
     * Here's a brief example implementation using the now-removed VuforiaLocalizer class:
     * <pre>
     * &#64;Override
     * public void getFrameBitamp(Continuation&lt;? extends Consumer&lt;Bitmap&gt;&gt; continuation) {
     *     vuforia.getFrameOnce(Continuation.createTrivial(new Consumer&lt;Frame&gt;() {
     *         &#64;Override
     *         public void accept(final Frame frame) {
     *             continuation.dispatch(new ContinuationResult&lt;Consumer&lt;Bitmap&gt;&gt;() {
     *                 &#64;Override
     *                 public void handle(Consumer&lt;Bitmap&gt; consumer) {
     *                     consumer.accept(vuforia.convertFrameToBitmap(frame));
     *                 &#125;
     *             &#125;);
     *         &#125;
     *     &#125;));
     * &#125;
     * </pre>
     *
     * @param continuation frame bitmap consumer continuation
     */
    // ### void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation);
}
