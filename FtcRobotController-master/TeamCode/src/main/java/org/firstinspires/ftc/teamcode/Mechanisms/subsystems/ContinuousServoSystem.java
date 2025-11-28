package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.PIDController;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;

public class ContinuousServoSystem implements Subsystem {
    private final CRServo servo1, servo2;
    private final DcMotor encoder1, encoder2;
    private final PIDController pid1, pid2;

    private double target1 = 0, target2 = 0;

    private static final double TICKS_PER_REV = 1120;
    private static final double SERVO_POWER = 0.8;
    private static final double KP = 0.01, KI = 0.0005, KD = 0.002;

    public ContinuousServoSystem(HardwareMap hardwareMap) {
        try {
            servo1 = hardwareMap.get(CRServo.class, "continuousServo1");
            servo2 = hardwareMap.get(CRServo.class, "continuousServo2");
            encoder1 = hardwareMap.get(DcMotor.class, "servoEncoder1");
            encoder2 = hardwareMap.get(DcMotor.class, "servoEncoder2");

            if (encoder1 != null) {
                encoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoder1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (encoder2 != null) {
                encoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoder2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            pid1 = new PIDController(KP, KI, KD);
            pid2 = new PIDController(KP, KI, KD);
        } catch (Exception e) {
            throw new RuntimeException("Continuous servo system initialization failed", e);
        }
    }

    @Override
    public void update() {
        if (servo1 != null && encoder1 != null) {
            double power = pid1.calculate(encoder1.getCurrentPosition(), target1);
            power = Math.max(-SERVO_POWER, Math.min(SERVO_POWER, power));
            servo1.setPower(power);
        }

        if (servo2 != null && encoder2 != null) {
            double power = pid2.calculate(encoder2.getCurrentPosition(), target2);
            power = Math.max(-SERVO_POWER, Math.min(SERVO_POWER, power));
            servo2.setPower(power);
        }
    }

    public void setPosition(int servo, double revolutions) {
        double target = revolutions * TICKS_PER_REV;
        if (servo == 1) target1 = target;
        else if (servo == 2) target2 = target;
    }

    @Override
    public void stop() {
        if (servo1 != null) servo1.setPower(0);
        if (servo2 != null) servo2.setPower(0);
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Servo1", "%.2f rev", getPositionRevolutions(1));
        telemetry.addData("Servo2", "%.2f rev", getPositionRevolutions(2));
    }

    // Getters
    public double getPositionRevolutions(int servo) {
        if (servo == 1 && encoder1 != null) return encoder1.getCurrentPosition() / TICKS_PER_REV;
        if (servo == 2 && encoder2 != null) return encoder2.getCurrentPosition() / TICKS_PER_REV;
        return 0;
    }

    public int getEncoderTicks(int servo) {
        if (servo == 1 && encoder1 != null) return encoder1.getCurrentPosition();
        if (servo == 2 && encoder2 != null) return encoder2.getCurrentPosition();
        return 0;
    }
}