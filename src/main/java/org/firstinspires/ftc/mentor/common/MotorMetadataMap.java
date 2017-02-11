package org.firstinspires.ftc.mentor.common;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by markdolecki on 12/19/16.
 */

public class MotorMetadataMap {
    public Map<MotorMetadata.MotorType, MotorMetadata> MOTOR_METADATA_MAP = new HashMap<>();

    public void initialize() {
        MOTOR_METADATA_MAP.put(MotorMetadata.MotorType.ANDYMARK_NEVEREST_3PT7, new MotorMetadata(
           "Andymark_Neverest_3.7",
                MotorMetadata.MotorType.ANDYMARK_NEVEREST_3PT7,
                12.0,
                1784.0,
                "3.7:1",
                482.16,
                "14W",
                "32.3 oz-in",
                "11.5 amps",
                "",
                ""
        ));
        MOTOR_METADATA_MAP.put(MotorMetadata.MotorType.ANDYMARK_NEVEREST_20, new MotorMetadata(
                "Andymark_Neverest_20",
                MotorMetadata.MotorType.ANDYMARK_NEVEREST_20,
                12.0,
                560.0,
                "20:1",
                275.0,
                "14W",
                "197 oz-in.",
                "11.5 amps",
                "",
                ""
        ));
        MOTOR_METADATA_MAP.put(MotorMetadata.MotorType.ANDYMARK_NEVEREST_40, new MotorMetadata(
                "Andymark_Neverest_40",
                MotorMetadata.MotorType.ANDYMARK_NEVEREST_40,
                12.0,
                1120.0,
                "40:1",
                129.0,
                "15W",
                "396 oz-in.",
                "11.5 amps",
                "1478 oz-in.",
                "12.8 oz-in."
                ));
        MOTOR_METADATA_MAP.put(MotorMetadata.MotorType.ANDYMARK_NEVEREST_60, new MotorMetadata(
                "Andymark_Neverest_60",
                MotorMetadata.MotorType.ANDYMARK_NEVEREST_60,
                12.0,
                1680.0,
                "60:1",
                105.0,
                "14W",
                "593 oz-in.",
                "11.5 amps",
                "",
                ""
                ));
        MOTOR_METADATA_MAP.put(MotorMetadata.MotorType.TETRIX, new MotorMetadata(
                "Tetrix",
                MotorMetadata.MotorType.TETRIX,
                12.0,
                1440.0,
                "",
                137.5,
                "",
                "3.9 kg-cm",
                "1.37 amps max",
                "",
                ""
                ));
        MOTOR_METADATA_MAP.put(MotorMetadata.MotorType.OTHER, new MotorMetadata(
                // Fill in metadata for a custom motor
        ));
        MOTOR_METADATA_MAP.put(MotorMetadata.MotorType.NONE, new MotorMetadata(
                // Use default values in MotorMetadata class
        ));
    }
}
