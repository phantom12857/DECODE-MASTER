package org.firstinspires.ftc.teamcode.Mechanisms.utils;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * The AprilTagDetector class manages all interactions with the FTC VisionPortal
 * for the purpose of detecting AprilTags.
 */
public class AprilTagDetector {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final String WEBCAM_NAME = "Webcam 1";

    // ==================================================
    // H A R D W A R E
    // ==================================================
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    // ==================================================
    // S T A T E
    // ==================================================
    private List<AprilTagDetection> currentDetections;

    /**
     * Constructor for AprilTagDetector.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public AprilTagDetector(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    /**
     * Updates the list of current AprilTag detections.
     * This should be called in the main robot loop.
     */
    public void update() {
        currentDetections = aprilTagProcessor.getDetections();
    }

    /**
     * Closes the vision portal and releases all resources.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Adds detailed AprilTag detection data to the telemetry.
     *
     * @param telemetry The Telemetry object.
     */
    @SuppressLint("DefaultLocale")
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("AprilTags Detected", getDetectionCount());

        if (currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n--- ID %d (%s) ---", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("Range: %.1f in", detection.ftcPose.range));
                    telemetry.addLine(String.format("Bearing: %.1f deg", detection.ftcPose.bearing));
                    telemetry.addLine(String.format("Yaw: %.1f deg", detection.ftcPose.yaw));
                }
            }
        }
    }

    // ==================================================
    // G E T T E R S
    // ==================================================

    /**
     * Gets the number of currently detected tags.
     */
    public int getDetectionCount() {
        return (currentDetections != null) ? currentDetections.size() : 0;
    }

    /**
     * Gets the distance to the closest backboard AprilTag.
     *
     * @param isBlueAlliance True if on the blue alliance, false for red.
     * @return The distance in inches, or -1 if no valid tag is found.
     */
    public double getBackboardDistance(boolean isBlueAlliance) {
        if (currentDetections == null) return -1;

        int[] targetTags = isBlueAlliance ? new int[]{1, 2, 3} : new int[]{4, 5, 6};
        double closestDistance = -1;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                for (int tagId : targetTags) {
                    if (detection.id == tagId) {
                        if (closestDistance == -1 || detection.ftcPose.range < closestDistance) {
                            closestDistance = detection.ftcPose.range;
                        }
                    }
                }
            }
        }
        return closestDistance;
    }
}
