package org.firstinspires.ftc.mentor.common;

import android.content.Context;
import android.os.Environment;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.Format;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import static android.os.Environment.DIRECTORY_DOCUMENTS;

/* DataLogger
 * A singleton class that is used to output formatted text data to a data file.
 * A new data file will be created each time based on the data and time.
 */
public class DataLogger {
    private Format formatter = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US);
    private Date date = new Date();
    private String filename = formatter.format(date) + ".log";
    private static DataLogger instance;
    private static FileOutputStream fileOutputStream;
    private static LinearOpMode linearOpMode;
    private Context context;
    private static PrintWriter printWriter;
    private static Locale locale = Locale.US;
    private static boolean DEBUG = false;
    private static boolean initialized = false;

    private DataLogger() {
        // Default constructor
    }

    static public DataLogger getInstance() {
        String functionName = "getInstance";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        if (instance == null) {
            instance = new DataLogger();
        }
        return instance;
    }

    public void initialize(LinearOpMode opMode) {
        linearOpMode = opMode;
        context = opMode.hardwareMap.appContext;
        String functionName = "initialize";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

//        if (DEBUG) {
//            DbgLog.msg("File directory = %s", context.getFilesDir());
//        }
//        context.getExternalFilesDir(DIRECTORY_DOCUMENTS);
        File path = Environment.getExternalStoragePublicDirectory(DIRECTORY_DOCUMENTS);

        path.mkdirs();

        File file = new File(path, filename);

//        try {
//            fileOutputStream = context. .openFileOutput(filename, Context.MODE_APPEND);
//        }
//        catch (FileNotFoundException e) {
//            e.printStackTrace();
//        }
//        printWriter = new PrintWriter(fileOutputStream);

        try {
            fileOutputStream = new FileOutputStream(file);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        printWriter = new PrintWriter(fileOutputStream);

        initialized = true;
    }

    public void logData(String format, Object... args) {
        String functionName = "logData";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        if (initialized) {
            printWriter.format(locale, format, args);
        }
        else {
            try {
                throw new Exception("DataLogger not initialized!");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void close() {
        String functionName = "close";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        if (initialized) {
            printWriter.flush();
            printWriter.close();
        }
        else {
            try {
                throw new Exception("DataLogger not initialized!");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }


}
