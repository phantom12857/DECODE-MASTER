package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PathCreator;

/**
 * Autonomous OpMode for the red alliance, starting on the far side.
 */
@SuppressWarnings("RedundantThrows")
@Autonomous(name = "Auto | Red Far Side", group = "red")
public class FarSideRed extends LinearOpMode {

    private DECODEMechanisms mechanisms;
    private AprilTagDetector aprilTagDetector;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // Initialization
            mechanisms = new DECODEMechanisms(hardwareMap);
            aprilTagDetector = new AprilTagDetector(hardwareMap);
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(PathCreator.startPoseFarSideRed);

            telemetry.addLine("Initialization Complete. Ready for start.");
            telemetry.update();

            waitForStart();

            // This auto is a placeholder and needs a path defined.
            // For now, it will just park.
            follower.followPath(PathCreator.createPathToPark(follower, PathCreator.startPoseFarSideRed));
            waitForPath();

        } finally {
            // Graceful shutdown
            if (mechanisms != null) mechanisms.stopAll();
            if (aprilTagDetector != null) aprilTagDetector.close();
        }
    }

    /**
     * Waits for the follower to complete its current path, with a timeout.
     */
    private void waitForPath() {
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && follower.isBusy() && (System.currentTimeMillis() - startTime) < (long) 5000) {
            follower.update();
            mechanisms.update();
        }
    }
}
