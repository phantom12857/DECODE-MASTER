// UPDATED: GoalSideBlue.java
package org.firstinspires.ftc.teamcode.autos;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.core.MechanismCoordinator;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PathCreator;

/**
 * Autonomous OpMode for the blue alliance, starting on the goal side.
 * <p>
 * REFACTORED: Added MechanismCoordinator for synchronized firing,
 * improved error handling, and comprehensive state management.
 */
@Autonomous(name = "Auto | Blue Goal Side (Refactored)", group = "blue")
public class GoalSideBlue extends LinearOpMode {

    // ==================================================
    // S U B S Y S T E M S
    // ==================================================
    private DECODEMechanisms mechanisms;
    private MechanismCoordinator coordinator;
    private AprilTagDetector aprilTagDetector;
    private Follower follower;

    // ==================================================
    // S T A T E   M A C H I N E
    // ==================================================
    private enum AutoState {
        INIT,
        HOMING_SPINDEXER,
        WAIT_FOR_HOMING,
        DRIVE_TO_SCORE,
        WAIT_FOR_PATH_1,
        FIRE_PRELOAD,
        WAIT_FOR_FIRE_1,
        DRIVE_TO_INTAKE_1,
        WAIT_FOR_PATH_2,
        INTAKE_BALLS_1,
        DRIVE_TO_SCORE_2,
        WAIT_FOR_PATH_3,
        FIRE_INTAKE_1,
        WAIT_FOR_FIRE_2,
        DRIVE_TO_PARK,
        WAIT_FOR_PARK,
        COMPLETE,
        ERROR_RECOVERY
    }

    private AutoState currentState = AutoState.INIT;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime runtime = new ElapsedTime();
    private String errorMessage = "";

    // ==================================================
    // T I M E O U T S
    // ==================================================
    private static final double HOMING_TIMEOUT_S = 5.0;
    private static final double PATH_TIMEOUT_S = 5.0;
    private static final double FIRE_TIMEOUT_S = 8.0;
    private static final double INTAKE_TIME_S = 3.0;
    private static final double LAUNCHER_SPINUP_TIME_S = 2.0;

    @SuppressWarnings({"RedundantThrows", "DuplicateBranchesInSwitch"})
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // ========== INITIALIZATION ==========
            telemetry.addLine("Initializing subsystems...");
            telemetry.update();

            mechanisms = new DECODEMechanisms(hardwareMap, telemetry);

            // Check for initialization errors
            if (!mechanisms.isInitialized()) {
                telemetry.addLine("⚠️ INITIALIZATION WARNINGS:");
                for (String error : mechanisms.getInitializationErrors()) {
                    telemetry.addLine("  " + error);
                }
                telemetry.addLine();
                telemetry.addLine("Continue anyway? Press START");
                telemetry.update();

                // Wait for confirmation or abort
                while (!isStarted() && !isStopRequested()) {
                    idle();
                }
            }

            // Initialize coordinator
            if (mechanisms.hasSpindexer() && mechanisms.hasLauncher()) {
                coordinator = new MechanismCoordinator(mechanisms.spindexer, mechanisms.launcher);
                telemetry.addLine("✓ MechanismCoordinator initialized");
            } else {
                telemetry.addLine("⚠️ MechanismCoordinator unavailable");
                coordinator = null;
            }

            // Initialize other systems
            aprilTagDetector = new AprilTagDetector(hardwareMap);
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(PathCreator.startPoseGoalSideBlue);

            // Build paths
            telemetry.addLine("Building paths...");
            telemetry.update();

            PathChain toScore = PathCreator.createPathToGoal(follower, PathCreator.startPoseGoalSideBlue);
            PathChain toIntake1 = PathCreator.createPathToIntake(follower, PathCreator.scorePose, PathCreator.intakePose1);
            PathChain toPark = PathCreator.createPathToPark(follower, PathCreator.scorePose);

            telemetry.addLine("✓ Initialization Complete");
            telemetry.addLine();
            telemetry.addLine("Ready for start!");
            telemetry.update();

            // ========== WAIT FOR START ==========
            waitForStart();
            runtime.reset();
            stateTimer.reset();

            // ========== AUTONOMOUS LOOP ==========
            while (opModeIsActive() && currentState != AutoState.COMPLETE) {
                // Update all systems
                mechanisms.update();
                follower.update();
                if (coordinator != null) {
                    coordinator.update();
                }

                // Execute current state
                switch (currentState) {
                    case INIT:
                        executeInit();
                        break;
                    case HOMING_SPINDEXER:
                        executeHomingSpindexer();
                        break;
                    case WAIT_FOR_HOMING:
                        executeWaitForHoming();
                        break;
                    case DRIVE_TO_SCORE:
                        executeDriveToScore(toScore);
                        break;
                    case WAIT_FOR_PATH_1:
                        executeWaitForPath("path to score");
                        break;
                    case FIRE_PRELOAD:
                        executeFirePreload();
                        break;
                    case WAIT_FOR_FIRE_1:
                        executeWaitForFire();
                        break;
                    case DRIVE_TO_INTAKE_1:
                        executeDriveToIntake(toIntake1);
                        break;
                    case WAIT_FOR_PATH_2:
                        executeWaitForPath("path to intake");
                        break;
                    case INTAKE_BALLS_1:
                        executeIntakeBalls();
                        break;
                    case DRIVE_TO_SCORE_2:
                        executeDriveToScore2(toScore);
                        break;
                    case WAIT_FOR_PATH_3:
                        executeWaitForPath("path to score 2");
                        break;
                    case FIRE_INTAKE_1:
                        executeFireIntake();
                        break;
                    case WAIT_FOR_FIRE_2:
                        executeWaitForFire();
                        break;
                    case DRIVE_TO_PARK:
                        executeDriveToPark(toPark);
                        break;
                    case WAIT_FOR_PARK:
                        executeWaitForPath("path to park");
                        break;
                    case ERROR_RECOVERY:
                        executeErrorRecovery(toPark);
                        break;
                    case COMPLETE:
                        // Do nothing, loop will exit
                        break;
                }

                // Telemetry
                updateTelemetry();
            }

        } catch (Exception e) {
            telemetry.addLine("🚨 CRITICAL ERROR:");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            throw e;
        } finally {
            // ========== CLEANUP ==========
            if (mechanisms != null) mechanisms.stopAll();
            if (aprilTagDetector != null) aprilTagDetector.close();

            telemetry.addLine("Autonomous complete");
            telemetry.update();
        }
    }

    // ==================================================
    // S T A T E   E X E C U T I O N   M E T H O D S
    // ==================================================

    private void executeInit() {
        if (mechanisms.isInitialized()) {
            transitionTo(AutoState.HOMING_SPINDEXER);
        } else {
            handleError("Mechanisms not initialized");
            transitionTo(AutoState.ERROR_RECOVERY);
        }
    }

    private void executeHomingSpindexer() {
        if (mechanisms.hasSpindexer()) {
            assert mechanisms.spindexer != null;
            mechanisms.spindexer.home();
            transitionTo(AutoState.WAIT_FOR_HOMING);
        } else {
            // Skip homing if spindexer not available
            transitionTo(AutoState.DRIVE_TO_SCORE);
        }
    }

    private void executeWaitForHoming() {
        assert mechanisms.spindexer != null;
        if (!mechanisms.spindexer.isBusy()) {
            transitionTo(AutoState.DRIVE_TO_SCORE);
        } else if (stateTimer.seconds() > HOMING_TIMEOUT_S) {
            handleError("Spindexer homing timeout");
            transitionTo(AutoState.DRIVE_TO_SCORE);  // Continue anyway
        }
    }

    private void executeDriveToScore(PathChain path) {
        follower.followPath(path);
        if (mechanisms.hasLauncher()) {
            assert mechanisms.launcher != null;
            mechanisms.launcher.setRPM(3250);
        }
        transitionTo(AutoState.WAIT_FOR_PATH_1);
    }

    private void executeDriveToScore2(PathChain path) {
        follower.followPath(path);
        transitionTo(AutoState.WAIT_FOR_PATH_3);
    }

    private void executeDriveToIntake(PathChain path) {
        follower.followPath(path);
        if (mechanisms.hasIntake()) {
            assert mechanisms.intake != null;
            mechanisms.intake.start();
        }
        transitionTo(AutoState.WAIT_FOR_PATH_2);
    }

    private void executeDriveToPark(PathChain path) {
        follower.followPath(path);
        transitionTo(AutoState.WAIT_FOR_PARK);
    }

    private void executeWaitForPath(String pathName) {
        if (!follower.isBusy()) {
            // Path complete, move to next state
            switch (currentState) {
                case WAIT_FOR_PATH_1:
                    transitionTo(AutoState.FIRE_PRELOAD);
                    break;
                case WAIT_FOR_PATH_2:
                    transitionTo(AutoState.INTAKE_BALLS_1);
                    break;
                case WAIT_FOR_PATH_3:
                    transitionTo(AutoState.FIRE_INTAKE_1);
                    break;
                case WAIT_FOR_PARK:
                    transitionTo(AutoState.COMPLETE);
                    break;
            }
        } else if (stateTimer.seconds() > PATH_TIMEOUT_S) {
            handleError(pathName + " timeout");
            transitionTo(AutoState.ERROR_RECOVERY);
        }
    }

    private void executeFirePreload() {
        if (coordinator != null) {
            // Use coordinator for synchronized firing
            if (coordinator.startMultiShotSequence(3)) {
                transitionTo(AutoState.WAIT_FOR_FIRE_1);
            } else {
                handleError("Could not start firing sequence");
                transitionTo(AutoState.ERROR_RECOVERY);
            }
        } else {
            // Fallback to manual firing
            executeFireManual(3);
            transitionTo(AutoState.DRIVE_TO_INTAKE_1);
        }
    }

    private void executeFireIntake() {
        if (coordinator != null) {
            if (coordinator.startMultiShotSequence(2)) {  // Likely got 2 balls from intake
                transitionTo(AutoState.WAIT_FOR_FIRE_2);
            } else {
                handleError("Could not start firing sequence");
                transitionTo(AutoState.DRIVE_TO_PARK);
            }
        } else {
            executeFireManual(2);
            transitionTo(AutoState.DRIVE_TO_PARK);
        }
    }

    private void executeWaitForFire() {
        if (coordinator != null && !coordinator.isBusy()) {
            if (coordinator.isInError()) {
                handleError("Firing error: " + coordinator.getLastError());
                // Continue anyway
            }

            // Firing complete, move to next state
            switch (currentState) {
                case WAIT_FOR_FIRE_1:
                    transitionTo(AutoState.DRIVE_TO_INTAKE_1);
                    break;
                case WAIT_FOR_FIRE_2:
                    transitionTo(AutoState.DRIVE_TO_PARK);
                    break;
            }
        } else if (stateTimer.seconds() > FIRE_TIMEOUT_S) {
            handleError("Firing sequence timeout");
            if (currentState == AutoState.WAIT_FOR_FIRE_1) {
                transitionTo(AutoState.DRIVE_TO_INTAKE_1);
            } else {
                transitionTo(AutoState.DRIVE_TO_PARK);
            }
        }
    }

    private void executeIntakeBalls() {
        // Just wait for intake time, mechanism already running
        if (stateTimer.seconds() > INTAKE_TIME_S) {
            if (mechanisms.hasIntake()) {
                assert mechanisms.intake != null;
                mechanisms.intake.stop();
            }
            transitionTo(AutoState.DRIVE_TO_SCORE_2);
        }
    }

    private void executeErrorRecovery(PathChain parkPath) {
        // Stop all mechanisms safely
        mechanisms.emergencyStop();

        // Try to park if time allows
        if (runtime.seconds() < 28.0) {  // Leave 2s buffer
            follower.followPath(parkPath);
            // Wait a moment for path to start
            sleep(500);
            transitionTo(AutoState.WAIT_FOR_PARK);
        } else {
            transitionTo(AutoState.COMPLETE);
        }
    }

    // ==================================================
    // H E L P E R   M E T H O D S
    // ==================================================

    /**
     * Manual firing fallback when coordinator is not available
     */
    private void executeFireManual(int shots) {
        if (!mechanisms.hasLauncher() || !mechanisms.hasSpindexer()) {
            return;
        }

        for (int i = 0; i < shots; i++) {
            // Wait for launcher to be at speed
            ElapsedTime waitTimer = new ElapsedTime();
            while (opModeIsActive()) {
                assert mechanisms.launcher != null;
                if (!(!mechanisms.launcher.isAtSpeed(100) &&
                        waitTimer.seconds() < LAUNCHER_SPINUP_TIME_S)) break;
                mechanisms.update();
                sleep(20);
            }

            // Fire
            assert mechanisms.launcher != null;
            mechanisms.launcher.kick();
            waitTimer.reset();
            while (opModeIsActive() &&
                    mechanisms.launcher.isKicking() &&
                    waitTimer.seconds() < 1.0) {
                mechanisms.update();
                sleep(20);
            }

            // Advance spindexer for next shot (if not last shot)
            if (i < shots - 1) {
                assert mechanisms.spindexer != null;
                mechanisms.spindexer.increment();
                waitTimer.reset();
                while (opModeIsActive() &&
                        mechanisms.spindexer.isBusy() &&
                        waitTimer.seconds() < 1.0) {
                    mechanisms.update();
                    sleep(20);
                }
            }
        }
    }

    /**
     * Transitions to a new state
     */
    private void transitionTo(AutoState newState) {
        currentState = newState;
        stateTimer.reset();
    }

    /**
     * Records an error message
     */
    private void handleError(String message) {
        errorMessage = message;
        System.err.println("Auto Error: " + message);
    }

    /**
     * Updates telemetry display
     */
    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.addData("State Time", "%.1f s", stateTimer.seconds());

        if (!errorMessage.isEmpty()) {
            telemetry.addData("⚠️ Last Error", errorMessage);
        }

        if (coordinator != null) {
            telemetry.addLine();
            coordinator.addTelemetryData(telemetry);
        }

        telemetry.update();
    }
}