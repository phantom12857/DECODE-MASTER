package org.firstinspires.ftc.teamcode.tests;/* package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "AutoYellowApproachFull")
public class AutoYellowApproachFull extends LinearOpMode {

    // Motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Camera
    private OpenCvCamera webcam;
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;

    // Vision tracking
    private double cX = 0, cY = 0, width = 0;
    public static final double OBJECT_WIDTH_INCHES = 3.75; // real-world width
    public static final double FOCAL_LENGTH = 728; // camera pixels

    // PID parameters
    private double kP_turn = 0.004;   // turning PID
    private double kP_strafe = 0.002; // lateral PID
    private double kP_forward = 0.005; // forward/back PID

    // Target distance in inches
    private static final double TARGET_DISTANCE = 2;

    @Override
    public void runOpMode() {

        // --- Hardware mapping ---
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // --- Camera setup ---
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        YellowBlobPipeline pipeline = new YellowBlobPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDevice();
        webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);

        telemetry.addLine("Ready for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // Get vision info
            cX = pipeline.getCenterX();
            cY = pipeline.getCenterY();
            width = pipeline.getBoundingWidth();

            double distance = estimateDistanceInches(width);

            // --- PID calculations ---
            double errorTurn = (cX - CAMERA_WIDTH / 2.0);
            double correctionTurn = kP_turn * errorTurn;

            double errorForward = distance - TARGET_DISTANCE;
            double correctionForward = kP_forward * errorForward;

            double errorStrafe = 0; // optional lateral correction if needed
            double correctionStrafe = kP_strafe * errorStrafe;

            // --- Obstacle detection with distance sensor ---
            boolean obstacleDetected = false;
            double obstacleAvoidTurn = 0;

            // Integrate distance sensor for obstacle detection
            // Check if we have a distance sensor available
            if (hardwareMap.tryGet(com.qualcomm.robotcore.hardware.DistanceSensor.class, "frontDistance") != null) {
                com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor = 
                    hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "frontDistance");
                
                double obstacleDistance = distanceSensor.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
                
                // If obstacle within 12 inches, take avoidance action
                if (obstacleDistance < 12.0) {
                    obstacleDetected = true;
                    // Turn away from obstacle (simple avoidance)
                    obstacleAvoidTurn = 0.3;  // Turn right to avoid
                    telemetry.addData("⚠️ Obstacle", "%.1f in", obstacleDistance);
                }
            }
            
            if (obstacleDetected) {
                // Simple avoidance: turn away from obstacle
                correctionTurn += obstacleAvoidTurn;
            }

            // Stop if close enough
            if (distance <= TARGET_DISTANCE || width == 0) {
                mecanumDrive(0, 0, 0);
                telemetry.addLine("Reached target or lost object");
                telemetry.update();
                break;
            } else {
                // Drive with PID corrections
                mecanumDrive(correctionForward, correctionTurn, correctionStrafe);
            }

            telemetry.addData("cX,cY", "(%.1f, %.1f)", cX, cY);
            telemetry.addData("Distance (in)", "%.2f", distance);
            telemetry.addData("Width (px)", "%.1f", width);
            telemetry.update();
        }

        mecanumDrive(0, 0, 0);
        webcam.stopStreaming();
    }

    private void mecanumDrive(double forward, double turn, double strafe) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        frontLeft.setPower(clamp(fl));
        frontRight.setPower(clamp(fr));
        backLeft.setPower(clamp(bl));
        backRight.setPower(clamp(br));
    }

    private double clamp(double val) {
        return Math.max(-1, Math.min(1, val));
    }

    private double estimateDistanceInches(double widthPixels) {
        if (widthPixels == 0) return Double.MAX_VALUE;
        return (OBJECT_WIDTH_INCHES * FOCAL_LENGTH) / widthPixels;
    }

    // --- Vision pipeline ---
    class YellowBlobPipeline extends OpenCvPipeline {
        private double cX = 0, cY = 0;
        private double width = 0;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(35, 255, 255);

            Mat mask = new Mat();
            Core.inRange(hsv, lowerYellow, upperYellow, mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double maxArea = 0;
            MatOfPoint largest = null;
            for (MatOfPoint c : contours) {
                double area = Imgproc.contourArea(c);
                if (area > maxArea) {
                    maxArea = area;
                    largest = c;
                }
            }

            if (largest != null) {
                Rect rect = Imgproc.boundingRect(largest);
                width = rect.width;
                cX = rect.x + rect.width / 2.0;
                cY = rect.y + rect.height / 2.0;

                Imgproc.rectangle(input, rect, new Scalar(255, 0, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
            } else {
                width = 0;
            }

            return input;
        }

        public double getCenterX() { return cX; }
        public double getCenterY() { return cY; }
        public double getBoundingWidth() { return width; }
    }
}
*/