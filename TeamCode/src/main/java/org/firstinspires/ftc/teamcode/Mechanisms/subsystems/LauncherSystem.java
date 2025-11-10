package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.PIDController;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;



public class LauncherSystem implements Subsystem {
    private final DcMotorEx launcher;
    private final Servo kicker;
    public final Servo light;

    private double targetRPM = 0;
    private boolean isKicking = false;
    private final PIDController pidController;

    private static final double MAX_RPM = 5000;
    private static final double TICKS_PER_REV = 28;
    private static final double KP = 0.02, KI = 0.001, KD = 0.005;

    public LauncherSystem(HardwareMap hardwareMap) {
        try {
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            kicker = hardwareMap.get(Servo.class, "kicker");
            light = hardwareMap.get(Servo.class, "light");

            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            kicker.setPosition(1.1); // Retracted position

            pidController = new PIDController(KP, KI, KD);
        } catch (Exception e) {
            throw new RuntimeException("Launcher system initialization failed", e);
        }
    }

    public void setRPM(double rpm) {
        targetRPM = Math.min(rpm, MAX_RPM);
    }

    @Override
    public void update() {
        if (targetRPM > 0) {
            double targetTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REV;
            double currentTicksPerSec = launcher.getVelocity();
            double correction = pidController.calculate(currentTicksPerSec, targetTicksPerSec);
            launcher.setVelocity(targetTicksPerSec + correction);
        } else {
            launcher.setPower(0);
        }
    }

    public void kick() {
        if (!isKicking) {
            isKicking = true;
            kicker.setPosition(0.67); // Extended position

            new Thread(() -> {
                try {
                    Thread.sleep(500);
                    kicker.setPosition(1.0); // Retract
                    Thread.sleep(750);
                    isKicking = false;
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }
    }

    @Override
    public void stop() {
        targetRPM = 0;
        if (launcher != null) {
            launcher.setPower(0);
        }
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Launcher", "Target: %.0f RPM, Actual: %.0f RPM",
                targetRPM, getActualRPM());
        telemetry.addData("Kicker", isKicking ? "KICKING" : "READY");
    }


    // Getters
    public double getTargetRPM() { return targetRPM; }
    public double getActualRPM() {
        return launcher != null ? (launcher.getVelocity() / TICKS_PER_REV) * 60.0 : 0;
    }
    public boolean isKicking() { return isKicking; }
    public boolean isAtSpeed(double toleranceRPM) {
        return Math.abs(getActualRPM() - targetRPM) <= toleranceRPM;
    }
}