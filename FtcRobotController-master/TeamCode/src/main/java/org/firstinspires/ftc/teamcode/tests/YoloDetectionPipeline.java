package org.firstinspires.ftc.teamcode.tests;/* package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;

public class YoloDetectionPipeline {

    private OpenCvCamera leftCam;
    private OpenCvCamera rightCam;
    private DetectionPipeline leftPipeline;
    private DetectionPipeline rightPipeline;
    private Telemetry telemetry;

    // Stereo baseline in inches
    private final double baseline = 16.0;
    // Focal length approximation (in pixels)
    private final double focalLengthPx = 800.0;

    public YoloDetectionPipeline(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "LeftCam"));
        rightCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "RightCam"));

        leftPipeline = new DetectionPipeline();
        rightPipeline = new DetectionPipeline();

        leftCam.setPipeline(leftPipeline);
        rightCam.setPipeline(rightPipeline);

        leftCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                leftCam.startStreaming(720, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("LeftCam Error", errorCode);
                telemetry.update();
            }
        });

        rightCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                rightCam.startStreaming(720, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("RightCam Error", errorCode);
                telemetry.update();
            }
        });

    }

    public Detection getNearestTarget() {
        List<Detection> leftDetections = leftPipeline.getDetections();
        List<Detection> rightDetections = rightPipeline.getDetections();

        if (leftDetections.isEmpty() || rightDetections.isEmpty()) return null;

        Detection nearest = null;
        double minDistance = Double.MAX_VALUE;

        for (Detection l : leftDetections) {
            for (Detection r : rightDetections) {
                // Match detections by vertical position (simple heuristic)
                if (Math.abs(l.boundingBox.y - r.boundingBox.y) < 50) {
                    double disparityPx = Math.abs(l.boundingBox.x - r.boundingBox.x);
                    if (disparityPx == 0) continue;

                    double distanceInches = (baseline * focalLengthPx) / disparityPx;

                    if (distanceInches < minDistance) {
                        minDistance = distanceInches;
                        nearest = new Detection(
                                l.boundingBox,
                                l.className,
                                l.confidence,
                                distanceInches,
                                l.getCenterX() // Approx angle offset if needed
                        );
                    }
                }
            }
        }

        if (nearest != null) {
            telemetry.addData("Nearest Distance", "%.2f in", nearest.distance);
            telemetry.update();
        }

        return nearest;
    }

    // --- Inner classes ---

    public static class Detection {
        public Rect boundingBox;
        public String className;
        public double confidence;
        public double distance;   // Stereo estimated
        public double angleOffset; // Approx pixel offset

        public Detection(Rect box, String name, double conf, double dist, double angleOffset) {
            this.boundingBox = box;
            this.className = name;
            this.confidence = conf;
            this.distance = dist;
            this.angleOffset = angleOffset;
        }

        public double getCenterX() {
            return boundingBox.x + boundingBox.width / 2.0;
        }
    }

    // Model-based detection pipeline (receives detections externally)
    public static class DetectionPipeline extends OpenCvPipeline {
        private List<Detection> detections = new ArrayList<>();

        // Call this from your main loop when you have model outputs
        public void setDetections(List<Detection> newDetections) {
            detections.clear();
            detections.addAll(newDetections);
        }

        @Override
        public Mat processFrame(Mat input) {
            // Simply return the input; detections are supplied externally
            return input;
        }

        public List<Detection> getDetections() {
            return new ArrayList<>(detections);
        }
    }
}
*/