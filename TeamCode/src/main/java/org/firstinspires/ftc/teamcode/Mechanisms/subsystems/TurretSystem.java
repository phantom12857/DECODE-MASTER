package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;

public class TurretSystem implements Subsystem {
    private final CRServo turret;
    private final AnalogInput encoder;

    public TurretSystem(HardwareMap hardwareMap) {
        try {
            turret = hardwareMap.get(CRServo.class, "turretSer");
            encoder = hardwareMap.get(AnalogInput.class, "turretEnc");
        } catch (Exception e) {
            throw new RuntimeException("Turret system initialization failed", e);
        }
    }

    public void setPower(double power) {
        double deadzone = 0.1;
        if (Math.abs(power) < deadzone) {
            power = 0;
        } else {
            power = Math.signum(power) * (0.4 + 0.6 * (Math.abs(power) - deadzone) / (1 - deadzone));
        }
        if (turret != null) {
            turret.setPower(Math.max(-1.0, Math.min(1.0, power)));
        }
    }

    @Override
    public void update() {
        // Turret typically doesn't need periodic updates
    }

    @Override
    public void stop() {
        setPower(0);
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Turret Voltage", "%.3fV", getEncoderVoltage());
    }

    public double getEncoderVoltage() {
        return encoder != null ? encoder.getVoltage() : 0;
    }
}