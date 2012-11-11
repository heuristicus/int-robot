package logging;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import launcher.RunParams;

public class Logger {

    private static final String LOG_DIRECTORY = RunParams.get("LOG_DIRECTORY");
    private final String logFile;
    private File file;
    private PrintWriter writer;
    private boolean autoFlush = false;

    public Logger() throws FileNotFoundException, IOException {
        this("ros_log_", true);
    }

    public Logger(String filename, boolean withDate)
            throws FileNotFoundException, IOException {
        if (withDate){
            logFile = filename + getTimeDateStamp() + ".log";
        } else {
            logFile = filename + ".log";
        }
        file = new File(LOG_DIRECTORY, logFile);
        writer = new PrintWriter(file);
        System.out.println("Initialised Logger to: " + file.getPath());
    }

    public void logString(String s) {
        writer.print(s);
        if (autoFlush) writer.flush();
    }

    /** Log string and add a newline */
    public void logLine(String s) {
        writer.println(s);
        if (autoFlush) writer.flush();
    }

    public void setAutoFlushing(boolean autoFlushing) {
        this.autoFlush = autoFlushing;
    }

    public void close() {
        writer.flush();
        writer.close();
    }

    public static String getTimeDateStamp(){
        Calendar curDate = Calendar.getInstance();
        DateFormat form = new SimpleDateFormat("dd_MM_yyyy-HH:mm:ss");
        return form.format(curDate.getTime());
    }

}