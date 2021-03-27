package developing;

import java.io.IOException;

import okhttp3.MediaType;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;
import okhttp3.Response;


public class HttpHandler {

    public void send(){
        OkHttpClient client = new OkHttpClient().newBuilder().build();
        MediaType mediaType = MediaType.parse("application/json");
        RequestBody body = RequestBody.create(mediaType, "{ \r\n    \"user\": \"Aahan\", \r\n    \"postDate\": \"2011-12-15\", \r\n    \"body\": \"Search is hard. Search should be easy.\" ,\r\n    \"title\": \"Ooga Booga\"\r\n}");
        Request request = new Request.Builder().url("http://127.0.0.1:9200/robot/data/1").method("POST", body).addHeader("Content-Type", "application/json").build();
        try {Response response = client.newCall(request).execute();} catch (IOException ignore) {}

    }

}
