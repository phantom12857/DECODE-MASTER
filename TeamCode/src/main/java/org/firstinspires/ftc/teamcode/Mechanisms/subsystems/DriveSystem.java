package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * The DriveSystem class manages the robot's mecanum drive train and orientation (IMU).
 * It handles field-centric and robot-centric driving modes and provides a thread-safe
 * way to access the robot's heading.
 */
public class DriveSystem implements Subsystem {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final long IMU_THREAD_SLEEP_MS = 20;
    private static final long SHUTDOWN_TIMEOUT_MS = 100;

    // ==================================================
    // H A R D W A R E
    // ==================================================
    private final DcMotor leftFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor rightBackDrive;
    private final BNO055IMU imu;

    // ==================================================
    // D R I V E   S T A T E
    // ==================================================
    private double headingOffset = 0.0;
    private boolean isFieldCentric = false;
    private volatile double cachedHeadingRadians = 0.0;
    private final ExecutorService imuExecutorService;

    /**
     * Constructor for DriveSystem.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public DriveSystem(HardwareMap hardwareMap) {
        // Drive Motor Initialization
        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

            leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            throw new RuntimeException("Drive system initialization failed", e);
        }

        // IMU Initialization and Background Thread
        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            params.mode = BNO055IMU.SensorMode.IMU;
            params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(params);
        } catch (Exception e) {
            throw new RuntimeException("IMU initialization failed", e);
        }
        
        imuExecutorService = Executors.newSingleThreadExecutor();
        imuExecutorService.submit(this::readImuContinuously);
    }

    /**
     * Continuously reads the IMU sensor data on a background thread.
     * This ensures the main loop is not blocked by sensor reads and that the heading is always fresh.
     */
    private void readImuContinuously() {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                cachedHeadingRadians = angles.firstAngle;
            } catch (Exception e) {
                // Ignore sensor read errors, as they can be transient.
            }

            try {
                Thread.sleep(IMU_THREAD_SLEEP_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Preserve the interrupted status
                break; // Exit loop
            }
        }
    }

    /**
     * Drives the robot using mecanum kinematics.
     *
     * @param y  The forward/backward power.
     * @param x  The strafing (left/right) power.
     * @param rx The rotational (turning) power.
     */
    public void driveMecanum(double y, double x, double rx) {
        double currentHeading = getHeadingRadians();
        double rotX = x;
        double rotY = y;

        if (isFieldCentric) {
            rotX = x * Math.cos(-currentHeading) - y * Math.sin(-currentHeading);
            rotY = x * Math.sin(-currentHeading) + y * Math.cos(-currentHeading);
        }

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double leftBackPower = (rotY - rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double rightBackPower = (rotY + rotX - rx) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);
    }

    @Override
    public void update() {
        // DriveSystem is primarily controlled by external calls to driveMecanum(),
        // so no periodic logic is needed here.
    }

    @Override
    public void stop() {
        driveMecanum(0, 0, 0); // Stop all motion
        if (imuExecutorService != null) {
            imuExecutorService.shutdownNow();
            try {
                if (!imuExecutorService.awaitTermination(SHUTDOWN_TIMEOUT_MS, TimeUnit.MILLISECONDS)) {
                    System.err.println("IMU thread did not terminate gracefully.");
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Drive Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Heading (Degrees)", "%.1f", Math.toDegrees(getHeadingRadians()));
    }

    // ==================================================
    // D R I V E   C O N F I G
    // ==================================================

    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    public void resetHeadingOffset() {
        headingOffset = cachedHeadingRadians;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }

    // ==================================================
    // G E T T E R S
    // ==================================================

    public double getHeadingRadians() {
        return cachedHeadingRadians - headingOffset;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(getHeadingRadians());
    }

    public boolean isFieldCentric() {
        return isFieldCentric;
    }
}
