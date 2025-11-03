package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;

public class DriveSystem implements Subsystem {
    private final DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private final BNO055IMU imu;
    private double headingOffset = 0;
    private boolean isFieldCentric = false;

    public DriveSystem(HardwareMap hardwareMap) {
        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            throw new RuntimeException("Drive system initialization failed", e);
        }

        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            params.mode = BNO055IMU.SensorMode.IMU;
            params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(params);
        } catch (Exception e) {
            throw new RuntimeException("IMU initialization failed", e);
        }
    }

    public void driveMecanum(double y, double x, double rx) {
        double heading = getHeadingRadians();

        if (isFieldCentric) {
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            x = rotX;
            y = rotY;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double lf = (y + x + rx) / denominator;
        double lb = (y - x + rx) / denominator;
        double rf = (y - x - rx) / denominator;
        double rb = (y + x - rx) / denominator;

        leftFrontDrive.setPower(lf);
        leftBackDrive.setPower(lb);
        rightFrontDrive.setPower(rf);
        rightBackDrive.setPower(rb);
    }

    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    public void resetHeadingOffset() {
        headingOffset = getHeadingRadians();
    }

    public boolean isFieldCentricActive() {
        return isFieldCentric;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(getHeadingRadians());
    }

    private double getHeadingRadians() {
        try {
            Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            return o.firstAngle - headingOffset;
        } catch (Exception e) {
            return 0;
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }

    @Override
    public void update() {
        // Drive typically doesn't need periodic updates
    }

    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Drive Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Heading", "%.1f°", getHeadingDegrees());
    }
}