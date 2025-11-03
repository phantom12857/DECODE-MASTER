package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.AprilTagDetector;
import org.firstinspires.ftc.teamcode.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.PathCallback;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class GoalSideBlue extends OpMode{
    private DECODEMechanisms mechanisms;
    private AprilTagDetector aprilTagDetector;
    private final ElapsedTime kickerTimer = new ElapsedTime();
    private final ElapsedTime spindexerTimer = new ElapsedTime();
    private double globalMaxPower = 0.8;
    private double intakingMaxPower = .225;
    private boolean launchInProgress = false;
    private int shotsFired = 0;
    private int targetSpindexerStep = 0;

    public enum LaunchAllState {
        Inactive,
        HomingSpindexer,
        ReadyToFire,
        Firing,
        WaitingForRetract,
        RotatingSpindexer,
        Complete
    }

    LaunchAllState launchAllState = LaunchAllState.Inactive;

    private double desiredRPM = 0.0;
    private final double REGRESSION_SLOPE = 13.98485;
    private final double REGRESSION_Y_INT = 2834.57576;
    private double distanceToApriltag = 0.0;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // ========== Poses ==========
    private final Pose startPose = new Pose(43.5, 134.75, Math.toRadians(90));
    private final Pose scanAndScorePLPose = new Pose(48, 98, Math.toRadians(130));
    private final Pose a3IntakePose = new Pose(52, 84, Math.toRadians(180));
    private final Pose a3EndPose = new Pose(25,84, Math.toRadians(180));
    private final Pose scorePose = new Pose(48,98, Math.toRadians(130));
    private final Pose a2IntakePose = new Pose(45, 61, Math.toRadians(180));
    private final Pose a2EndPose = new Pose(20,61, Math.toRadians(180));
    private final Pose a2ControlPose = new Pose(60, 60, Math.toRadians(0));
    private final Pose parkPose = new Pose(35, 70, Math.toRadians(180));

    // ==========  Paths  ==========
    private PathChain scanAndScorePL, toIntakeA3, intakingA3, scoreA3, toIntakeA2, intakingA2, scoreA2, park;

    public void buildPaths(){
        scanAndScorePL = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scanAndScorePLPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scanAndScorePLPose.getHeading())
                .build();

        toIntakeA3 = follower.pathBuilder()
                .addPath(new BezierLine(scanAndScorePLPose, a3IntakePose))
                .setLinearHeadingInterpolation(scanAndScorePLPose.getHeading(), a3IntakePose.getHeading())
                .build();

        intakingA3 = follower.pathBuilder()
                .addPath(new BezierLine(a3IntakePose, a3EndPose))
                .setLinearHeadingInterpolation(a3IntakePose.getHeading(), a3EndPose.getHeading())
                .build();

        scoreA3 = follower.pathBuilder()
                .addPath(new BezierLine(a3EndPose, scorePose))
                .setLinearHeadingInterpolation(a3EndPose.getHeading(), scorePose.getHeading())
                .build();

        toIntakeA2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, a2IntakePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), a2IntakePose.getHeading())
                .build();

        intakingA2 = follower.pathBuilder()
                .addPath(new BezierLine(a2IntakePose, a2EndPose))
                .setLinearHeadingInterpolation(a2IntakePose.getHeading(), a2EndPose.getHeading())
                .build();

        scoreA2 = follower.pathBuilder()
                .addPath(new BezierCurve(a2EndPose, a2ControlPose, scorePose))
                .setLinearHeadingInterpolation(a2EndPose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    // ========== Main Auto Switch Case ==========
    public void autonomousPathUpdate(){
        switch(pathState) {
            case 0:
                // Move to scoring position and set up launcher
                if (!follower.isBusy()) {
                    follower.setMaxPower(globalMaxPower);
                    follower.followPath(scanAndScorePL, true);
                    mechanisms.setLauncherRPM(3250);
                    mechanisms.startIntake();
                    mechanisms.reverseIntake();
                    shotsFired = 0;
                    pathState = 1;
                }
                break;

            case 1:
                // Wait for launcher to get up to speed and launch 3 pixels
                if (!follower.isBusy()) {
                    if (launchAllState == LaunchAllState.Inactive) {
                        // Start by homing the spindexer to ensure we know its position
                        launchAllState = LaunchAllState.HomingSpindexer;
                        mechanisms.homeSpindexer();
                    } else if (launchAllState == LaunchAllState.Complete) {
                        // All 3 shots are done, continue to next path
                        launchAllState = LaunchAllState.Inactive;
                        pathState = 2;
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(.5);
                    mechanisms.ballsLoaded = 0;
                    follower.followPath(toIntakeA3, false);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(intakingMaxPower);
                    follower.followPath(intakingA3, false);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(globalMaxPower);
                    follower.followPath(scoreA3, true);
                    shotsFired = 0;
                    pathState = 5;
                }
                break;

            case 5:
                // Wait for launcher to get up to speed and launch 3 pixels
                if (!follower.isBusy()) {
                    if (launchAllState == LaunchAllState.Inactive) {
                        shotsFired = 0;
                        // Start by homing the spindexer to ensure we know its position
                        launchAllState = LaunchAllState.HomingSpindexer;
                        mechanisms.homeSpindexer();
                    } else if (launchAllState == LaunchAllState.Complete) {
                        // All 3 shots are done, continue to next path
                        launchAllState = LaunchAllState.Inactive;
                        pathState = 6;
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(toIntakeA2, false);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(intakingMaxPower);
                    follower.followPath(intakingA2, false);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()){
                    follower.setMaxPower(globalMaxPower);
                    follower.followPath(scoreA2, true);
                    shotsFired = 0;
                    pathState = 9;
                }
                break;

            case 9:
                // Wait for launcher to get up to speed and launch 3 pixels
                if (!follower.isBusy()) {
                    if (launchAllState == LaunchAllState.Inactive) {
                        // Start by homing the spindexer to ensure we know its position
                        launchAllState = LaunchAllState.HomingSpindexer;
                        mechanisms.homeSpindexer();
                    } else if (launchAllState == LaunchAllState.Complete) {
                        // All 3 shots are done, continue to next path
                        launchAllState = LaunchAllState.Inactive;
                        pathState = 10;
                    }
                }
                break;

            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(park, false);
                    mechanisms.stopAllMotors();
                    pathState = -1;
                }
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        mechanisms.updateAllSystems();
        launchAll();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Launch All State", launchAllState);
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("Current Spindexer Step", mechanisms.getSpindexerStep());
        telemetry.addData("Target Spindexer Step", targetSpindexerStep);
        telemetry.addData("Spindexer Moving", mechanisms.isSpindexerMoving());
        telemetry.addData("Kicker Timer", kickerTimer.milliseconds());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        mechanisms = new DECODEMechanisms(hardwareMap);
        aprilTagDetector = new AprilTagDetector(hardwareMap);

        telemetry.addLine("Initialized, waiting for start...");
        telemetry.addLine("Spindexer limit switch: " +
                (mechanisms.isSpindexerLimitPressed() ? "PRESSED" : "Released"));
        telemetry.addLine("Auto-Intake: ENABLED by default");
        telemetry.addLine("Hood PID Holding: ACTIVE");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("DO NOT RUN WITH BATTERY BELOW 13 VOLTS!!!");
        telemetry.addLine("IF YOU DO I WILL FIND YOU :)");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathState = 0;
        shotsFired = 0;
        targetSpindexerStep = 0;
    }


    public void launchAll(){
        switch (launchAllState){
            case Inactive:
                // Do nothing, waiting to start
                break;

            case HomingSpindexer:
                // Wait for spindexer to finish homing
                if (!mechanisms.isSpindexerMoving()) {
                    // Spindexer is now homed and at position 0
                    // Ready to fire first shot without any rotation
                    targetSpindexerStep = 0;
                    launchAllState = LaunchAllState.ReadyToFire;
                }
                break;

            case ReadyToFire:
                // Wait for spindexer to be completely stopped before firing
                if (!mechanisms.isSpindexerMoving()) {
                    // Start the kick sequence
                    mechanisms.fireKicker();
                    launchInProgress = true;
                    launchAllState = LaunchAllState.Firing;
                    kickerTimer.reset();
                }
                break;

            case Firing:
                // Wait for kick to complete (extend time)
                if (kickerTimer.milliseconds() > 500) {
                    mechanisms.retractKicker();
                    launchAllState = LaunchAllState.WaitingForRetract;
                    kickerTimer.reset();
                }
                break;

            case WaitingForRetract:
                // Wait for retraction to complete
                if (kickerTimer.milliseconds() > 300) {
                    shotsFired++;
                    launchInProgress = false;

                    if (shotsFired >= 3) {
                        launchAllState = LaunchAllState.Complete;
                    } else {
                        // Calculate next spindexer step (1, 2 for remaining shots)
                        targetSpindexerStep = shotsFired; // 0, 1, 2 for 3 shots
                        mechanisms.setSpindexerStep(targetSpindexerStep);
                        launchAllState = LaunchAllState.RotatingSpindexer;
                    }
                }
                break;

            case RotatingSpindexer:
                // Wait for spindexer to finish rotating to the target step
                if (!mechanisms.isSpindexerMoving()) {
                    // Spindexer is stopped, safe to fire again
                    launchAllState = LaunchAllState.ReadyToFire;
                }
                break;

            case Complete:
                // All three shots completed
                break;
        }
    }
}