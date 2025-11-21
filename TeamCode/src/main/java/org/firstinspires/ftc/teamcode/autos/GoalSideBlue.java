package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PathCreator;

/**
 * Autonomous OpMode for the blue alliance, starting on the goal side.
 */
@Autonomous(name = "Auto | Blue Goal Side", group = "blue")
public class GoalSideBlue extends LinearOpMode {

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
            follower.setStartingPose(PathCreator.startPoseGoalSideBlue);

            // Build Paths
            PathChain toScore = PathCreator.createPathToGoal(follower, PathCreator.startPoseGoalSideBlue);
            PathChain toIntake1 = PathCreator.createPathToIntake(follower, PathCreator.scorePose, PathCreator.intakePose1);
            PathChain toIntake2 = PathCreator.createPathToIntake(follower, PathCreator.scorePose, PathCreator.intakePose2);
            PathChain toPark = PathCreator.createPathToPark(follower, PathCreator.scorePose);

            telemetry.addLine("Initialization Complete. Ready for start.");
            telemetry.update();

            waitForStart();

            // Autonomous Sequence
            mechanisms.spindexer.home();
            follower.followPath(toScore);
            mechanisms.launcher.setRPM(3250);
            waitForPath(2000);

            fireThreeRings();

            follower.followPath(toIntake1);
            mechanisms.intake.start();
            waitForPath(3000);
            mechanisms.intake.stop();

            follower.followPath(toScore);
            waitForPath(2000);
            fireThreeRings();

            follower.followPath(toPark);
            waitForPath(5000);

        } finally {
            // Graceful shutdown
            if (mechanisms != null) mechanisms.stopAll();
            if (aprilTagDetector != null) aprilTagDetector.close();
        }
    }

    /**
     * Waits for the follower to complete its current path, with a timeout.
     *
     * @param timeoutMs The maximum time to wait in milliseconds.
     */
    private void waitForPath(long timeoutMs) {
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && follower.isBusy() && (System.currentTimeMillis() - startTime) < timeoutMs) {
            follower.update();
            mechanisms.update();
        }
    }

    /**
     * Fires three rings, waiting for the launcher and spindexer between shots.
     */
    private void fireThreeRings() {
        for (int i = 0; i < 3; i++) {
            // Wait for launcher to be at speed
            long startTime = System.currentTimeMillis();
            while (opModeIsActive() && !mechanisms.launcher.isAtSpeed(100) && (System.currentTimeMillis() - startTime) < 2000) {
                mechanisms.update();
            }

            // Fire
            mechanisms.launcher.kick();
            startTime = System.currentTimeMillis();
            while (opModeIsActive() && mechanisms.launcher.isKicking() && (System.currentTimeMillis() - startTime) < 1000) {
                mechanisms.update();
            }

            // Advance spindexer for the next shot (if not the last shot)
            if (i < 2) {
                mechanisms.spindexer.increment();
                startTime = System.currentTimeMillis();
                while (opModeIsActive() && mechanisms.spindexer.isBusy() && (System.currentTimeMillis() - startTime) < 1000) {
                    mechanisms.update();
                }
            }
        }
    }
}
