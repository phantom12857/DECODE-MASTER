package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.PIDController;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;

public class HoodSystem implements Subsystem {
    private final CRServo hood;
    private final AnalogInput encoder;
    private final PIDController pidController;

    private double targetPosition = 0.5;
    private boolean isHoldingPosition = false;
    private double manualPower = 0;

    private static final double MIN_POS = 0.1;
    private static final double MAX_POS = 1.0;
    private static final double POSITION_TOLERANCE = 0.02;
    private static final double HOLD_POWER_LIMIT = 0.3;
    private static final double KP = 0.8, KI = 0.05, KD = 0.1;

    public HoodSystem(HardwareMap hardwareMap) {
        try {
            hood = hardwareMap.get(CRServo.class, "hood");
            encoder = hardwareMap.get(AnalogInput.class, "hoodEnc");
            pidController = new PIDController(KP, KI, KD);

            // Initialize position from encoder
            targetPosition = getEncoderPosition();
        } catch (Exception e) {
            throw new RuntimeException("Hood system initialization failed", e);
        }
    }

    public void setPower(double power) {
        double deadzone = 0.1;

        if (Math.abs(power) < deadzone) {
            // Position holding mode
            if (!isHoldingPosition) {
                targetPosition = getEncoderPosition();
                isHoldingPosition = true;
                pidController.reset();
            }

            double currentPos = getEncoderPosition();
            double pidPower = pidController.calculate(currentPos, targetPosition);
            pidPower = Math.max(-HOLD_POWER_LIMIT, Math.min(HOLD_POWER_LIMIT, pidPower));

            if (Math.abs(currentPos - targetPosition) > POSITION_TOLERANCE) {
                hood.setPower(pidPower);
            } else {
                hood.setPower(0);
            }

            manualPower = 0;
        } else {
            // Manual control mode
            isHoldingPosition = false;
            manualPower = power;

            double scaledPower = Math.signum(power) * (0.3 + 0.7 * (Math.abs(power) - deadzone) / (1 - deadzone));
            hood.setPower(Math.max(-1.0, Math.min(1.0, scaledPower)));
            targetPosition = getEncoderPosition();
        }
    }

    public void setPosition(double position) {
        targetPosition = Math.max(MIN_POS, Math.min(MAX_POS, position));
        isHoldingPosition = true;
        pidController.reset();
    }

    @Override
    public void update() {
        // Hood PID is handled automatically in setHoodPower
        // This method can be used for additional hood logic if needed
    }

    @Override
    public void stop() {
        setPower(0);
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Hood Position", "%.3f", getPosition());
        telemetry.addData("Hood Target", "%.3f", targetPosition);
        telemetry.addData("Hood Holding", isHoldingPosition ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Hood Voltage", "%.3fV", getEncoderVoltage());
    }

    private double getEncoderPosition() {
        return encoder != null ? Math.max(0.0, Math.min(1.0, encoder.getVoltage() / 3.3)) : 0.5;
    }

    // Getters
    public double getPosition() { return getEncoderPosition(); }
    public double getTargetPosition() { return targetPosition; }
    public boolean isHoldingPosition() { return isHoldingPosition; }
    public double getEncoderVoltage() { return encoder != null ? encoder.getVoltage() : 0; }
}