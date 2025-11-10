package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;

public class IntakeSystem implements Subsystem {
    private final DcMotor intake;

    public IntakeSystem(HardwareMap hardwareMap) {
        try {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            throw new RuntimeException("Intake system initialization failed", e);
        }
    }

    public void start() {
        setPower(1.0);
    }
    public void passiveIntake(){setPower(.4);}

    public void reverse() {
        setPower(-1.0);
    }


    public void stop() {
        setPower(0);
    }

    public void setPower(double power) {
        if (intake != null) {
            intake.setPower(power);
        }
    }

    public double getPower() {
        return intake != null ? intake.getPower() : 0;
    }

    @Override
    public void update() {
        // Intake typically doesn't need periodic updates
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Intake Power", "%.2f", getPower());
    }

    public boolean isActiveIntake(boolean activeIntake){
        if(intake.getPower() > 0.4){
            return true;
        }
        else return false;
    }
}