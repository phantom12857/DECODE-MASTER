package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;

@TeleOp
public class intakeTest extends LinearOpMode {
    private DECODEMechanisms mechanisms;
    private boolean intake = true;

    @Override
    public void runOpMode() {
        // Initialize mechanisms
        mechanisms = new DECODEMechanisms(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            handleIntakeControls();
        }

    }

    private void handleIntakeControls() {
        if (gamepad2.right_trigger > 0.1) {
            mechanisms.startIntake();
        } else if (gamepad2.left_trigger > 0.1) {
            mechanisms.reverseIntake();
        } else {
            mechanisms.stopIntake();
        }
    }
}
