package org.firstinspires.ftc.mentor.common;

/**
 * Created by markdolecki on 12/19/16.
 */

// Generic exception for robot configuration errors
public class RobotConfigurationException extends Exception {

    /**
     * Constructs a new {@code Exception} that includes the current stack trace.
     */
    public RobotConfigurationException() {

        super();
    }

    /**
     * Constructs a new {@code Exception} with the current stack trace and the
     * specified detail message.
     *
     * @param detailMessage the detail message for this exception.
     */
    public RobotConfigurationException(String detailMessage) {

        super(detailMessage);
    }

    /**
     * Constructs a new {@code Exception} with the current stack trace, the
     * specified detail message and the specified cause.
     *
     * @param detailMessage the detail message for this exception.
     * @param throwable
     */
    public RobotConfigurationException(String detailMessage, Throwable throwable) {
        super(detailMessage, throwable);
    }

    /**
     * Constructs a new {@code Exception} with the current stack trace and the
     * specified cause.
     *
     * @param throwable the cause of this exception.
     */
    public RobotConfigurationException(Throwable throwable) {
        super(throwable);
    }

}
