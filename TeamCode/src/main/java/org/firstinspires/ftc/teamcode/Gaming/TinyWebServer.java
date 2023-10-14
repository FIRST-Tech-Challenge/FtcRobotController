/*
 * The MIT License
 *
 * Copyright 2018 Sonu Auti http://sonuauti.com twitter @SonuAuti
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package org.firstinspires.ftc.teamcode.Gaming;

import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.lang.reflect.Method;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.net.URL;
import java.net.URLDecoder;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Locale;
import java.util.Map;
import java.util.TimeZone;
import java.util.regex.Pattern;
// Taken from https://github.com/sonuauti/Android-Web-Server/blob/master/AndroidWebServer/src/androidhttpweb/TinyWebServer.java
/**
 *
 * @author Sonu Auti @cis
 */
public class TinyWebServer extends Thread {

    /**
     * @param args the command line arguments
     */
    private static ServerSocket serverSocket;
    private final Map<String, String> lowerCaseHeader = new HashMap<>();

    public static String CONTENT_TYPE = "text/html";
    private String CONTENT_DATE = "";
    private String CONN_TYPE = "";
    private String Content_Encoding = "";
    private String content_length = "";
    private String STATUS = "200";
    private boolean keepAlive = true;
    private String SERVER_NAME = "Firefly http server v0.1";
    private static final String MULTIPART_FORM_DATA_HEADER = "multipart/form-data";
    private static final String ASCII_ENCODING = "US-ASCII";
    private String REQUEST_TYPE = "GET";
    private String HTTP_VER = "HTTP/1.1";

    //all status
    public static String PAGE_NOT_FOUND = "404";
    public static String OKAY = "200";
    public static String CREATED = "201";
    public static String ACCEPTED = "202";
    public static String NO_CONTENT = "204";
    public static String PARTIAL_NO_CONTENT = "206";
    public static String MULTI_STATUS = "207";
    public static String MOVED_PERMANENTLY = "301";
    public static String SEE_OTHER = "303";
    public static String NOT_MODIFIED = "304";
    public static String TEMP_REDIRECT = "307";
    public static String BAD_REQUEST = "400";
    public static String UNAUTHORIZED_REQUEST = "401";
    public static String FORBIDDEN = "403";
    public static String NOT_FOUND = "404";
    public static String METHOD_NOT_ALLOWED = "405";
    public static String NOT_ACCEPTABLE = "406";
    public static String REQUEST_TIMEOUT = "408";
    public static String CONFLICT = "409";
    public static String GONE = "410";
    public static String LENGTH_REQUIRED = "411";
    public static String PRECONDITION_FAILED = "412";

    public static String PAYLOAD_TOO_LARGE = "413";
    public static String UNSUPPORTED_MEDIA_TYPE = "415";
    public static String RANGE_NOT_SATISFIABLE = "416";
    public static String EXPECTATION_FAILED = "417";
    public static String TOO_MANY_REQUESTS = "429";

    public static String INTERNAL_ERROR = "500";
    public static String NOT_IMPLEMENTED = "501";
    public static String SERVICE_UNAVAILABLE = "503";
    public static String UNSUPPORTED_HTTP_VERSION = "505";

    public static final String CONTENT_DISPOSITION_REGEX = "([ |\t]*Content-Disposition[ |\t]*:)(.*)";

    public static final Pattern CONTENT_DISPOSITION_PATTERN = Pattern.compile(CONTENT_DISPOSITION_REGEX, Pattern.CASE_INSENSITIVE);

    public static final String CONTENT_TYPE_REGEX = "([ |\t]*content-type[ |\t]*:)(.*)";

    public static final Pattern CONTENT_TYPE_PATTERN = Pattern.compile(CONTENT_TYPE_REGEX, Pattern.CASE_INSENSITIVE);

    public static final String CONTENT_DISPOSITION_ATTRIBUTE_REGEX = "[ |\t]*([a-zA-Z]*)[ |\t]*=[ |\t]*['|\"]([^\"^']*)['|\"]";

    public static final Pattern CONTENT_DISPOSITION_ATTRIBUTE_PATTERN = Pattern.compile(CONTENT_DISPOSITION_ATTRIBUTE_REGEX);

    public static final String CONTENT_LENGTH_REGEX = "Content-Length:";
    public static final Pattern CONTENT_LENGTH_PATTERN = Pattern.compile(CONTENT_LENGTH_REGEX, Pattern.CASE_INSENSITIVE);

    public static final String USER_AGENT = "User-Agent:";
    public static final Pattern USER_AGENT_PATTERN = Pattern.compile(USER_AGENT, Pattern.CASE_INSENSITIVE);

    public static final String HOST_REGEX = "Host:";
    public static final Pattern CLIENT_HOST_PATTERN = Pattern.compile(HOST_REGEX, Pattern.CASE_INSENSITIVE);

    public static final String CONNECTION_TYPE_REGEX = "Connection:";
    public static final Pattern CONNECTION_TYPE_PATTERN = Pattern.compile(CONNECTION_TYPE_REGEX, Pattern.CASE_INSENSITIVE);

    public static final String ACCEPT_ENCODING_REGEX = "Accept-Encoding:";
    public static final Pattern ACCEPT_ENCODING_PATTERN = Pattern.compile(ACCEPT_ENCODING_REGEX, Pattern.CASE_INSENSITIVE);

    private static final String CONTENT_REGEX = "[ |\t]*([^/^ ^;^,]+/[^ ^;^,]+)";

    private static final Pattern MIME_PATTERN = Pattern.compile(CONTENT_REGEX, Pattern.CASE_INSENSITIVE);

    private static final String CHARSET_REGEX = "[ |\t]*(charset)[ |\t]*=[ |\t]*['|\"]?([^\"^'^;^,]*)['|\"]?";

    private static final Pattern CHARSET_PATTERN = Pattern.compile(CHARSET_REGEX, Pattern.CASE_INSENSITIVE);

    private static final String BOUNDARY_REGEX = "[ |\t]*(boundary)[ |\t]*=[ |\t]*['|\"]?([^\"^'^;^,]*)['|\"]?";

    private static final Pattern BOUNDARY_PATTERN = Pattern.compile(BOUNDARY_REGEX, Pattern.CASE_INSENSITIVE);


    public static String WEB_DIR_PATH="/";
    public static String SERVER_IP="localhost";
    public static int SERVER_PORT=9000;
    public static boolean isStart=true;
    public static String INDEX_FILE_NAME="index.html";


    public TinyWebServer(final String ip, final int port) throws IOException {

        InetAddress addr = InetAddress.getByName(ip); ////"172.31.0.186");
        serverSocket = new ServerSocket(port, 100, addr);
        serverSocket.setSoTimeout(5000);  //set timeout for listener

    }

    @Override
    public void run() {

        while (isStart) {
            try {
                //wait for new connection on port 5000
                Socket newSocket = serverSocket.accept();
                Thread newClient = new EchoThread(newSocket);
                newClient.start();
            } catch (SocketTimeoutException s) {
            } catch (IOException e) {
            }

        }//endof Never Ending while loop

    }

    public class EchoThread extends Thread {

        protected Socket socket;
        protected boolean nb_open;

        public EchoThread(Socket clientSocket) {
            this.socket = clientSocket;
            this.nb_open = true;
        }

        @Override
        public void run() {

            try {
                DataInputStream in = null;
                DataOutputStream out = null;

                if (socket.isConnected()) {
                    in = new DataInputStream(socket.getInputStream());
                    out = new DataOutputStream(socket.getOutputStream());
                }

                byte[] data = new byte[1500];
                //socket.setSoTimeout(60 * 1000 * 5);

                while (in.read(data) != -1) {
                    String recData = new String(data).trim();
                    //System.out.println("received data: \n" + recData);
                    //System.out.println("------------------------------");
                    String[] header = recData.split("\\r?\\n");

                    String contentLen = "0";
                    String contentType = "text/html";
                    String connectionType = "keep-alive";
                    String hostname = "";
                    String userAgent = "";
                    String encoding = "";

                    String[] h1 = header[0].split(" ");
                    if (h1.length == 3) {
                        setRequestType(h1[0]);
                        setHttpVer(h1[2]);
                    }

                    for (int h = 0; h < header.length; h++) {
                        String value = header[h].trim();

                        //System.out.println(header[h]+" -> "+CONTENT_LENGTH_PATTERN.matcher(header[h]).find());
                        if (CONTENT_LENGTH_PATTERN.matcher(value).find()) {
                            contentLen = value.split(":")[1].trim();
                        } else if (CONTENT_TYPE_PATTERN.matcher(value).find()) {
                            contentType = value.split(":")[1].trim();
                        } else if (CONNECTION_TYPE_PATTERN.matcher(value).find()) {
                            connectionType = value.split(":")[1].trim();
                        } else if (CLIENT_HOST_PATTERN.matcher(value).find()) {
                            hostname = value.split(":")[1].trim();
                        } else if (USER_AGENT_PATTERN.matcher(value).find()) {
                            for (String ua : value.split(":")) {
                                if (!ua.equalsIgnoreCase("User-Agent:")) {
                                    userAgent += ua.trim();
                                }
                            }
                        } else if (ACCEPT_ENCODING_PATTERN.matcher(value).find()) {
                            encoding = value.split(":")[1].trim();
                        }

                    }

                    if (!REQUEST_TYPE.equals("")) {
                        String postData = "";
                        if (REQUEST_TYPE.equalsIgnoreCase("POST") && !contentLen.equals("0")) {
                            postData = header[header.length - 1];
                            if (postData.length() > 0 && contentLen.length() > 0) {
                                int len = Integer.valueOf(contentLen);
                                postData = postData.substring(0, len);
                                // System.out.println("Post data -> " + contentLen + " ->" + postData);
                            }
                        }

                        // System.out.println("contentLen ->" + contentLen + "\ncontentType ->" + contentType + "\nhostname ->" + hostname + "\nconnectionType-> " + connectionType + "\nhostname ->" + hostname + "\nuserAgent -> " + userAgent);
                        final String requestLocation = h1[1];
                        if (requestLocation != null) {
                            processLocation(out, requestLocation, postData);
                        }
                        //System.out.println("requestLocation "+requestLocation);
                    }

                }
            } catch (Exception er) {
                er.printStackTrace();
            }

        }

    }

    public void processLocation(DataOutputStream out, String location, String postData) {

        String data = "";
        switch (location) {
            case "/":
                //root location, server index file
                CONTENT_TYPE = "text/html";
                data = "<!DOCTYPE html><html> <head> <title>Robot Keyboard & Mouse TeleOp</title> <style>.splitimg{width: 11.15%; height: 33.5%;}</style> </head> <body> <script async> var pressedKeys={\"q\": false,\"w\": false,\"e\": false,\"r\": false,\"t\": false,\"y\": false,\"u\": false,\"i\": false,\"o\": false,\"p\": false,\"a\": false,\"s\": false,\"d\": false,\"f\": false,\"g\": false,\"h\": false,\"j\": false,\"k\": false,\"l\": false,\"z\": false,\"x\": false,\"c\": false,\"v\": false,\"b\": false,\"n\": false,\"m\": false, targetheight: \"none\"}; function base64toBlob(base64Data, contentType){contentType=contentType || ''; var sliceSize=1024; var byteCharacters=atob(base64Data); var bytesLength=byteCharacters.length; var slicesCount=Math.ceil(bytesLength / sliceSize); var byteArrays=new Array(slicesCount); for (var sliceIndex=0; sliceIndex < slicesCount; ++sliceIndex){var begin=sliceIndex * sliceSize; var end=Math.min(begin + sliceSize, bytesLength); var bytes=new Array(end - begin); for (var offset=begin, i=0; offset < end; ++i, ++offset){bytes[i]=byteCharacters[offset].charCodeAt(0);}byteArrays[sliceIndex]=new Uint8Array(bytes);}return new Blob(byteArrays,{type: contentType});}window.addEventListener(\"keydown\", function(event){if(pressedKeys.hasOwnProperty(event.key)){pressedKeys[event.key]=true;}document.getElementById(\"jsonoutput\").innerText=JSON.stringify(pressedKeys);}); window.addEventListener(\"keyup\", function(event){if(pressedKeys.hasOwnProperty(event.key)){pressedKeys[event.key]=false;}document.getElementById(\"jsonoutput\").innerText=JSON.stringify(pressedKeys);}); async function sendToServer(){let res=await fetch(\"192.168.43.1:6969/simplepostendpoint\",{method: \"post\", headers:{'Content-Type': 'application/json'}, body: JSON.stringify(pressedKeys)}); pressedKeys.targetheight=\"none\"; return res;}var theIntervalthingy=setInterval(sendToServer, 50); async function getthecamimg(){let res=await fetch(\"192.168.43.1:6969/curimgdata\"); let textoutput=document.getElementById(\"textoutput\"); let imgoutput=document.getElementById(\"imgoutput\"); textoutput.innerText=res.body; imgoutput.src=\"data:image/jpeg;base64,\" + res.body;}function GoToHeight(daheight){pressedKeys.targetheight=daheight; document.getElementById(\"jsonoutput\").innerText=JSON.stringify(pressedKeys);}</script> <h3>Just press keys!</h3> <p id=\"jsonoutput\"></p><button onclick=\"getthecamimg();\">Click me for camera image stuff.</button> <p id=\"textoutput\"></p><h3>Go to a height:</h3> <img onclick=\"GoToHeight('ground');\" src=\"data:image/svg+xml;charset=UTF-8,%3Csvg%20width%3D%22150%22%20height%3D%22100%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%20150%20100%22%20preserveAspectRatio%3D%22none%22%3E%0A%20%20%20%20%20%20%3Cdefs%3E%0A%20%20%20%20%20%20%20%20%3Cstyle%20type%3D%22text%2Fcss%22%3E%0A%20%20%20%20%20%20%20%20%20%20%23holder%20text%20%7B%0A%20%20%20%20%20%20%20%20%20%20%20%20fill%3A%20%23000000%3B%0A%20%20%20%20%20%20%20%20%20%20%20%20font-family%3A%20sans-serif%3B%0A%20%20%20%20%20%20%20%20%20%20%20%20font-size%3A%2020px%3B%0A%20%20%20%20%20%20%20%20%20%20%20%20font-weight%3A%20400%3B%0A%20%20%20%20%20%20%20%20%20%20%7D%0A%20%20%20%20%20%20%20%20%3C%2Fstyle%3E%0A%20%20%20%20%20%20%3C%2Fdefs%3E%0A%20%20%20%20%20%20%3Cg%20id%3D%22holder%22%3E%0A%20%20%20%20%20%20%20%20%3Crect%20width%3D%22100%25%22%20height%3D%22100%25%22%20fill%3D%22%23ffffff%22%3E%3C%2Frect%3E%0A%20%20%20%20%20%20%20%20%3Cg%3E%0A%20%20%20%20%20%20%20%20%20%20%3Ctext%20text-anchor%3D%22middle%22%20x%3D%2250%25%22%20y%3D%2250%25%22%20dy%3D%22.3em%22%3EThe%20ground.%20%3A%7C%3C%2Ftext%3E%0A%20%20%20%20%20%20%20%20%3C%2Fg%3E%0A%20%20%20%20%20%20%3C%2Fg%3E%0A%20%20%20%20%3C%2Fsvg%3E\"></img> <img style=\"width: 6.45%; height: 13.35%;\" onclick=\"GoToHeight('collect');\" src=\"data:image/jpeg;base64,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\"></img> <img class=\"splitimg\" onclick=\"GoToHeight('lowpole');\" src=\"data:image/jpeg;base64,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\"></img> <img class=\"splitimg\" onclick=\"GoToHeight('mediumpole');\" src=\"data:image/jpeg;base64,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\"></img> <img class=\"splitimg\" onclick=\"GoToHeight('highpole');\" src=\"data:image/jpeg;base64,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\"></img> </body></html>";
//                data=readFile(WEB_DIR_PATH+"/"+INDEX_FILE_NAME);
                constructHeader(out, data.length() + "", data);
                break;
            default:

                System.out.println("url location -> " + location);
                URL geturl = getDecodedUrl("http://localhost" + location);
                String[] dirPath = geturl.getPath().split("/");
                String fullFilePath=geturl.getPath();
                if (dirPath.length > 1) {
                    String fileName = dirPath[dirPath.length - 1];
                    HashMap qparms = (HashMap) splitQuery(geturl.getQuery());
                    if(REQUEST_TYPE.equals("POST")){
                        if (qparms==null){ qparms=new HashMap<String,String>();}
                        qparms.put("_POST", postData);
                    }
                    //System.out.println("File name " + fileName);
                    //System.out.println("url parms " + qparms);
                    CONTENT_TYPE = getContentType(fileName);
                    if(!CONTENT_TYPE.equals("text/plain")){
                        // System.out.println("Full file path - >"+fullFilePath +" "+CONTENT_TYPE);

                        if(CONTENT_TYPE.equals("image/jpeg") || CONTENT_TYPE.equals("image/png") || CONTENT_TYPE.equals("video/mp4")){
                            byte[] bytdata=readImageFiles(WEB_DIR_PATH+fullFilePath,CONTENT_TYPE);
                            //System.out.println(bytdata.length);
                            if(bytdata!=null){
                                constructHeaderImage(out, bytdata.length+"", bytdata);
                            }else{
                                pageNotFound();
                            }
                        }else{
                            data=readFile(WEB_DIR_PATH+fullFilePath);
                            if(!data.equals("")){
                                constructHeader(out, data.length() + "", data);
                            }else{
                                pageNotFound();
                            }
                        }
                    }else{
                        data = getResultByName(fileName, qparms);
                        constructHeader(out, data.length() + "", data);
                    }


                }

        }

    }

    public URL getDecodedUrl(String parms) {
        try {
            //String decodedurl =URLDecoder.decode(parms,"UTF-8");
            URL aURL = new URL(parms);
            return aURL;
        } catch (Exception er) {
        }
        return null;
    }

    public static HashMap<String, String> splitQuery(String parms) {
        try {
            final HashMap<String, String> query_pairs = new HashMap<>();
            final String[] pairs = parms.split("&");
            for (String pair : pairs) {
                final int idx = pair.indexOf("=");
                final String key = idx > 0 ? URLDecoder.decode(pair.substring(0, idx), "UTF-8") : pair;
                if (!query_pairs.containsKey(key)) {
                    query_pairs.put(key, "");
                }
                final String value = idx > 0 && pair.length() > idx + 1 ? URLDecoder.decode(pair.substring(idx + 1), "UTF-8") : null;
                query_pairs.put(key, value);
            }
            return query_pairs;
        } catch (Exception er) {
        }
        return null;
    }

    public String getResultByName(String name, HashMap qparms) {
        try {
            String ClassName = "org.firstinspires.ftc.teamcode.AppApis";
            Class<?> rClass = Class.forName(ClassName); // convert string classname to class
            Object obj = rClass.newInstance();          // invoke empty constructor
            Method getNameMethod = obj.getClass().getMethod(name, HashMap.class);
            STATUS = TinyWebServer.OKAY;
            return getNameMethod.invoke(obj, qparms).toString();
        } catch (Exception er) {
            // er.printStackTrace();
            return pageNotFound();
        }
    }

    public void setRequestType(String type) {
        // System.out.println("REQUEST TYPE " + type);
        this.REQUEST_TYPE = type;
    }

    public void setHttpVer(String httpver) {
        // System.out.println("REQUEST ver " + httpver);
        this.HTTP_VER = httpver;
    }

    public String getRequestType() {
        return this.REQUEST_TYPE;
    }

    public String getHttpVer() {
        return this.HTTP_VER;
    }

    public String pageNotFound() {
        STATUS = NOT_FOUND;
        CONTENT_TYPE = "text/html";
        //customize your page here
        return "<!DOCTYPE html>"
                + "<html><head><title>Page not found | Weird robot web server. (Firefly web server)</title>"
                + "</head><body><h3>Requested page not found :(</h3></body></html>";
    }

    //hashtable initilization for content types
    static Hashtable<String, String> mContentTypes = new Hashtable();

    {
        mContentTypes.put("js", "application/javascript");
        mContentTypes.put("php", "text/html");
        mContentTypes.put("java", "text/html");
        mContentTypes.put("json", "application/json");
        mContentTypes.put("png", "image/png");
        mContentTypes.put("jpg", "image/jpeg");
        mContentTypes.put("html", "text/html");
        mContentTypes.put("css", "text/css");
        mContentTypes.put("mp4", "video/mp4");
        mContentTypes.put("mov", "video/quicktime");
        mContentTypes.put("wmv", "video/x-ms-wmv");

    }

    //get request content type
    public static String getContentType(String path) {
        String type = tryGetContentType(path);
        if (type != null) {
            return type;
        }
        return "text/plain";
    }

    //get request content type from path
    public static String tryGetContentType(String path) {
        int index = path.lastIndexOf(".");
        if (index != -1) {
            String e = path.substring(index + 1);
            String ct = mContentTypes.get(e);
            // System.out.println("content type: " + ct);
            if (ct != null) {
                return ct;
            }
        }
        return null;
    }

    private void constructHeader(DataOutputStream output, String size, String data) {
        SimpleDateFormat gmtFrmt = new SimpleDateFormat("E, d MMM yyyy HH:mm:ss 'GMT'", Locale.US);
        gmtFrmt.setTimeZone(TimeZone.getTimeZone("GMT"));
        PrintWriter pw = new PrintWriter(new BufferedWriter(new OutputStreamWriter(output)), false);
        pw.append("HTTP/1.1 ").append(STATUS).append(" \r\n");
        if (this.CONTENT_TYPE != null) {
            printHeader(pw, "Content-Type", this.CONTENT_TYPE);
        }
        printHeader(pw, "Date", gmtFrmt.format(new Date()));
        printHeader(pw, "Connection", (this.keepAlive ? "keep-alive" : "close"));
        printHeader(pw, "Content-Length", size);
        printHeader(pw, "Server", SERVER_NAME);
        pw.append("\r\n");
        pw.append(data);
        pw.flush();
        //pw.close();
    }

    private void constructHeaderImage(DataOutputStream output, String size, byte[] data) {
        try{

            SimpleDateFormat gmtFrmt = new SimpleDateFormat("E, d MMM yyyy HH:mm:ss 'GMT'", Locale.US);
            gmtFrmt.setTimeZone(TimeZone.getTimeZone("GMT"));
            PrintWriter pw = new PrintWriter(new BufferedWriter(new OutputStreamWriter(output)), false);
            pw.append("HTTP/1.1 ").append(STATUS).append(" \r\n");
            if (this.CONTENT_TYPE != null) {
                printHeader(pw, "Content-Type", this.CONTENT_TYPE);
            }
            printHeader(pw, "Date", gmtFrmt.format(new Date()));
            printHeader(pw, "Connection", (this.keepAlive ? "keep-alive" : "close"));
            printHeader(pw, "Content-Length", size);
            printHeader(pw, "Server", SERVER_NAME);
            pw.append("\r\n");
            pw.flush();
            output.write(data);
            output.flush();
            //System.out.println("data sent success");

            //pw.close();
        }catch(Exception er){er.printStackTrace();}

    }


    @SuppressWarnings("static-method")
    protected void printHeader(PrintWriter pw, String key, String value) {
        pw.append(key).append(": ").append(value).append("\r\n");
    }

    public byte[] readImageFiles(String fileName,String filetype){
        try{
            File ifile=new File(fileName);
            if(ifile.exists()){
                if(filetype.equalsIgnoreCase("image/png") || filetype.equalsIgnoreCase("image/jpeg") || filetype.equalsIgnoreCase("image/gif") || filetype.equalsIgnoreCase("image/jpg")){
                    FileInputStream fis = new FileInputStream(fileName);
                    byte[] buffer = new byte[fis.available()];
                    while (fis.read(buffer) != -1) {}
                    fis.close();
                    return buffer;
                }
            }else{

            }
        }catch(Exception er){}
        return null;
    }
    public String readFile(String fileName){
        String content="";
        try{
            File ifile=new File(fileName);
            if(ifile.exists()){
                FileInputStream fis = new FileInputStream(fileName);
                byte[] buffer = new byte[10];
                StringBuilder sb = new StringBuilder();
                while (fis.read(buffer) != -1) {
                    sb.append(new String(buffer));
                    buffer = new byte[10];
                }
                fis.close();
                content = sb.toString();
            }else{
                pageNotFound();
                return content;
            }
        }catch(Exception er){
            pageNotFound();
            return "";
        }
        return content;
    }


    public static void init(String ip,int port,String public_dir){

        SERVER_IP=ip;
        SERVER_PORT=port;
        WEB_DIR_PATH=public_dir;
        scanFileDirectory();

    }

    public static void startServer(String ip,int port,String public_dir){
        try {

            isStart=true;
            init(ip,port,public_dir);
            Thread t = new TinyWebServer(SERVER_IP, SERVER_PORT);
            t.start();
            System.out.println("Server Started !");

        } catch (IOException e) {
            e.printStackTrace();
        } catch (Exception e) {
        }
    }

    public static void stopServer(){
        if(isStart){
            try{
                isStart=false;
                serverSocket.close();
                System.out.println("Server stopped running !");
            }catch(IOException er){
                er.printStackTrace();
            }
        }
    }


    //scan for index file
    public static void scanFileDirectory(){
        boolean isIndexFound=false;
        try{
            File file=new File(WEB_DIR_PATH);
            if(file.isDirectory()){
                File[] allFiles=file.listFiles();
                for (File allFile : allFiles) {
                    //System.out.println(allFile.getName().split("\\.")[0]);
                    if(allFile.getName().split("\\.")[0].equalsIgnoreCase("index")){
                        TinyWebServer.INDEX_FILE_NAME=allFile.getName();
                        isIndexFound=true;
                    }
                }
            }

        }catch(Exception er){}

        if(!isIndexFound){
            System.out.println("Index file not found !");
        }
    }

   /* //use for testing
    public static void main(String[] args) {
        try {

            Thread t = new TinyWebServer(SERVER_IP, SERVER_PORT);
            t.start();
            System.out.println("Server Started !");

        } catch (IOException e) {
            e.printStackTrace();
        } catch (Exception e) {
        }
    }*/

}