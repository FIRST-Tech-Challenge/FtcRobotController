package org.firstinspires.ftc.teamcode.Gaming;

import java.io.IOException
import java.net.InetAddress
import java.net.ServerSocket
import java.util.regex.Pattern

abstract class HTTPServer: Thread() {
    private var serverSocket: ServerSocket? = null
    private val lowerCaseHeader: Map<String, String> = HashMap()

    var CONTENT_TYPE = "text/html"
    private val CONTENT_DATE = ""
    private val CONN_TYPE = ""
    private val Content_Encoding = ""
    private val content_length = ""
    private val STATUS = "200"
    private val keepAlive = true
    private val SERVER_NAME = "Firefly http server v0.1"
    private val MULTIPART_FORM_DATA_HEADER = "multipart/form-data"
    private val ASCII_ENCODING = "US-ASCII"
    private val REQUEST_TYPE = "GET"
    private val HTTP_VER = "HTTP/1.1"

    //all status
    var PAGE_NOT_FOUND = "404"
    var OKAY = "200"
    var CREATED = "201"
    var ACCEPTED = "202"
    var NO_CONTENT = "204"
    var PARTIAL_NO_CONTENT = "206"
    var MULTI_STATUS = "207"
    var MOVED_PERMANENTLY = "301"
    var SEE_OTHER = "303"
    var NOT_MODIFIED = "304"
    var TEMP_REDIRECT = "307"
    var BAD_REQUEST = "400"
    var UNAUTHORIZED_REQUEST = "401"
    var FORBIDDEN = "403"
    var NOT_FOUND = "404"
    var METHOD_NOT_ALLOWED = "405"
    var NOT_ACCEPTABLE = "406"
    var REQUEST_TIMEOUT = "408"
    var CONFLICT = "409"
    var GONE = "410"
    var LENGTH_REQUIRED = "411"
    var PRECONDITION_FAILED = "412"

    var PAYLOAD_TOO_LARGE = "413"
    var UNSUPPORTED_MEDIA_TYPE = "415"
    var RANGE_NOT_SATISFIABLE = "416"
    var EXPECTATION_FAILED = "417"
    var TOO_MANY_REQUESTS = "429"

    var INTERNAL_ERROR = "500"
    var NOT_IMPLEMENTED = "501"
    var SERVICE_UNAVAILABLE = "503"
    var UNSUPPORTED_HTTP_VERSION = "505"

    val CONTENT_DISPOSITION_REGEX = "([ |\t]*Content-Disposition[ |\t]*:)(.*)"

    val CONTENT_DISPOSITION_PATTERN =
        Pattern.compile(CONTENT_DISPOSITION_REGEX, Pattern.CASE_INSENSITIVE)

    val CONTENT_TYPE_REGEX = "([ |\t]*content-type[ |\t]*:)(.*)"

    val CONTENT_TYPE_PATTERN = Pattern.compile(CONTENT_TYPE_REGEX, Pattern.CASE_INSENSITIVE)

    val CONTENT_DISPOSITION_ATTRIBUTE_REGEX =
        "[ |\t]*([a-zA-Z]*)[ |\t]*=[ |\t]*['|\"]([^\"^']*)['|\"]"

    val CONTENT_DISPOSITION_ATTRIBUTE_PATTERN = Pattern.compile(CONTENT_DISPOSITION_ATTRIBUTE_REGEX)

    val CONTENT_LENGTH_REGEX = "Content-Length:"
    val CONTENT_LENGTH_PATTERN = Pattern.compile(CONTENT_LENGTH_REGEX, Pattern.CASE_INSENSITIVE)

    val USER_AGENT = "User-Agent:"
    val USER_AGENT_PATTERN = Pattern.compile(USER_AGENT, Pattern.CASE_INSENSITIVE)

    val HOST_REGEX = "Host:"
    val CLIENT_HOST_PATTERN = Pattern.compile(HOST_REGEX, Pattern.CASE_INSENSITIVE)

    val CONNECTION_TYPE_REGEX = "Connection:"
    val CONNECTION_TYPE_PATTERN = Pattern.compile(CONNECTION_TYPE_REGEX, Pattern.CASE_INSENSITIVE)

    val ACCEPT_ENCODING_REGEX = "Accept-Encoding:"
    val ACCEPT_ENCODING_PATTERN = Pattern.compile(ACCEPT_ENCODING_REGEX, Pattern.CASE_INSENSITIVE)

    private val CONTENT_REGEX = "[ |\t]*([^/^ ^;^,]+/[^ ^;^,]+)"

    private val MIME_PATTERN = Pattern.compile(CONTENT_REGEX, Pattern.CASE_INSENSITIVE)

    private val CHARSET_REGEX = "[ |\t]*(charset)[ |\t]*=[ |\t]*['|\"]?([^\"^'^;^,]*)['|\"]?"

    private val CHARSET_PATTERN = Pattern.compile(CHARSET_REGEX, Pattern.CASE_INSENSITIVE)

    private val BOUNDARY_REGEX = "[ |\t]*(boundary)[ |\t]*=[ |\t]*['|\"]?([^\"^'^;^,]*)['|\"]?"

    private val BOUNDARY_PATTERN = Pattern.compile(BOUNDARY_REGEX, Pattern.CASE_INSENSITIVE)

    var WEB_DIR_PATH = "/"
    var SERVER_IP = "localhost"
    var SERVER_PORT = 9000
    var isStart = true
    var INDEX_FILE_NAME = "index.html"


    @Throws(IOException::class)
    open fun TinyWebServer(ip: String?, port: Int) {
        val addr = InetAddress.getByName(ip) ////"172.31.0.186");
        this.serverSocket = ServerSocket(port, 100, addr)
        this.serverSocket!!.soTimeout = 5000 //set timeout for listner
    }
}
