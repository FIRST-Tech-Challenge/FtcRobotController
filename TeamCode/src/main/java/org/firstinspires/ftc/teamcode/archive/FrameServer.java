package org.firstinspires.ftc.teamcode;

import java.io.ByteArrayInputStream;

import fi.iki.elonen.NanoHTTPD;

class FrameServer extends NanoHTTPD {
    public FrameServer() {
        super(8080); // Start server on port 8080
    }

    @Override
    public Response serve(IHTTPSession session) {
        byte[] imageBytes = new byte[0];
        /*
        if (!latestFrame.empty()) {
            Mat jpegFrame = new Mat();
            Imgcodecs.imencode(".jpg", latestFrame, jpegFrame);
            imageBytes = new byte[(int) (jpegFrame.total() * jpegFrame.elemSize())];
            jpegFrame.get(0, 0, imageBytes);
            jpegFrame.release(); // Release the temporary matrix
        }*/

        return newFixedLengthResponse(Response.Status.OK, "image/jpeg", new ByteArrayInputStream(imageBytes), imageBytes.length);
    }
}
