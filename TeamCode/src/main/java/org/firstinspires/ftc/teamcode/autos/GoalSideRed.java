package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PathCreator;

/**
 * Autonomous OpMode for the red alliance, starting on the goal side.
 */
@Autonomous(name = "Auto | Red Goal Side", group = "red")
public class GoalSideRed extends LinearOpMode {

    private DECODEMechanisms mechanisms;
    private AprilTagDetector aprilTagDetector;
    private Follower follower;

    @SuppressWarnings({"RedundantThrows", "unused"})
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // Initialization
            mechanisms = new DECODEMechanisms(hardwareMap);
            aprilTagDetector = new AprilTagDetector(hardwareMap);
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(PathCreator.startPoseGoalSideRed);

            // Build Paths
            PathChain toScore = PathCreator.createPathToGoal(follower, PathCreator.startPoseGoalSideRed);
            PathChain toIntake1 = PathCreator.createPathToIntake(follower, PathCreator.scorePose, PathCreator.intakePose1);
            PathChain toPark = PathCreator.createPathToPark(follower, PathCreator.scorePose);

            telemetry.addLine("Initialization Complete. Ready for start.");
            telemetry.update();

            waitForStart();

            // Autonomous Sequence
            assert mechanisms.spindexer != null;
            mechanisms.spindexer.home();
            follower.followPath(toScore);
            assert mechanisms.launcher != null;
            mechanisms.launcher.setRPM(3250);
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
            long startTime = System.currentTimeMillis();
            while (opModeIsActive()) {
                assert mechanisms.launcher != null;
                if (!(!mechanisms.launcher.isAtSpeed(100) && (System.currentTimeMillis() - startTime) < 2000))
                    break;
                mechanisms.update();
            }

            assert mechanisms.launcher != null;
            mechanisms.launcher.kick();
            startTime = System.currentTimeMillis();
            while (opModeIsActive() && mechanisms.launcher.isKicking() && (System.currentTimeMillis() - startTime) < 1000) {
                mechanisms.update();
            }

            if (i < 2) {
                assert mechanisms.spindexer != null;
                mechanisms.spindexer.increment();
                startTime = System.currentTimeMillis();
                while (opModeIsActive() && mechanisms.spindexer.isBusy() && (System.currentTimeMillis() - startTime) < 1000) {
                    mechanisms.update();
                }
            }
        }
    }
}
