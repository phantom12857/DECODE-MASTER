package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class FarSideBlue extends OpMode{
    private DECODEMechanisms mechanisms;
    private AprilTagDetector aprilTagDetector;
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime kickerTimer = new ElapsedTime();
    private final ElapsedTime waitTimer = new ElapsedTime();
    private double globalMaxPower = 0.95;
    private double intakingMaxPower = 0.35;
    private int shotsFired = 0;
    private double minLauncherVelocity = 4190;
    private double maxLauncherVelocity = 4215;

    public enum LaunchAllState {
        Inactive,
        ReadyToFire,
        Firing,
        WaitingForRetract,
        RotatingSpindexer,
        Complete
    }

    LaunchAllState launchAllState = LaunchAllState.Inactive;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // ========== Poses ==========
    private final Pose startPose = new Pose(51.75, 8.625, Math.toRadians(90));
    private final Pose pLPose = new Pose(51.75, 14, Math.toRadians(104));
    private final Pose a1IntakePose = new Pose(45, 33, Math.toRadians(180));
    private final Pose a1EndPose = new Pose(15, 33, Math.toRadians(180));
    private final Pose scorePose = new Pose(51.75,14, Math.toRadians(112));
    private final Pose bIntakePose = new Pose(20,25, Math.toRadians(218));
    private final Pose bIntakeControlPose = new Pose(40, 25, Math.toRadians(225));
    private final Pose bEndPose = new Pose(14,10, Math.toRadians(218));
    private final Pose parkPose = new Pose(44, 30, Math.toRadians(90));

    // ==========  Paths  ==========
    private PathChain scorePL, toIntakeA1, intakingA1, scoreA1, toIntakeB, intakingB, scoreB, park;
    public void buildPaths(){
        scorePL = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pLPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pLPose.getHeading())
                .build();

        toIntakeA1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, a1IntakePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), a1IntakePose.getHeading())
                .build();

        intakingA1 = follower.pathBuilder()
                .addPath(new BezierLine(a1IntakePose, a1EndPose))
                .setLinearHeadingInterpolation(a1IntakePose.getHeading(), a1EndPose.getHeading())
                .build();

        scoreA1 = follower.pathBuilder()
                .addPath(new BezierLine(a1EndPose, scorePose))
                .setLinearHeadingInterpolation(a1EndPose.getHeading(), scorePose.getHeading())
                .build();

        toIntakeB = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,bIntakeControlPose, bIntakePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), bIntakePose.getHeading())
                .build();

        intakingB = follower.pathBuilder()
                .addPath(new BezierLine(bIntakePose, bEndPose))
                .setLinearHeadingInterpolation(bIntakePose.getHeading(), bEndPose.getHeading())
                .build();

        scoreB = follower.pathBuilder()
                .addPath(new BezierLine(bEndPose, scorePose))
                .setLinearHeadingInterpolation(bEndPose.getHeading(), scorePose.getHeading())
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
                    follower.followPath(scorePL, true);
                    mechanisms.spindexer.autoIntakeEnabled = false;
                    //mechanisms.hood.setPosition(.4);
                    mechanisms.launcher.setRPM(maxLauncherVelocity);
                    mechanisms.intake.passiveIntake();
                    mechanisms.spindexer.setBallsLoaded(3);
                    shotsFired = 0;
                    pathState = 1;
                }
                break;

            case 1:
                // Wait for launcher to get up to speed and launch 3 pixels
                if (!follower.isBusy()) {
                    if (launchAllState == LaunchAllState.Inactive && !mechanisms.spindexer.isMoving()) {
                        launchAllState = LaunchAllState.ReadyToFire;
                    } else if (launchAllState == LaunchAllState.Complete) {
                        launchAllState = LaunchAllState.Inactive;
                        pathState = 2;
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    mechanisms.spindexer.setBallsLoaded(0);
                    mechanisms.spindexer.home();
                    mechanisms.spindexer.autoIntakeEnabled = true;
                    mechanisms.intake.start();
                    follower.followPath(toIntakeA1, false);
                    intakeTimer.reset();
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(intakingMaxPower);
                    follower.followPath(intakingA1, false);
                    if(intakeTimer.seconds() > 5){
                        waitTimer.reset();
                        pathState = 4;
                    }
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(globalMaxPower);
                    mechanisms.spindexer.autoIntakeEnabled = false;
                    mechanisms.intake.passiveIntake();
                    follower.followPath(scoreA1, true);
                    shotsFired = 0;
                    pathState = 5;
                }
                break;

            case 5:
                // Wait for launcher to get up to speed and launch 3 pixels
                if (!follower.isBusy() && waitTimer.seconds() > 2.75) {
                    if (launchAllState == LaunchAllState.Inactive) {
                        shotsFired = 0;
                        launchAllState = LaunchAllState.ReadyToFire;
                    } else if (launchAllState == LaunchAllState.Complete) {
                        launchAllState = LaunchAllState.Inactive;
                        pathState = 10;
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    mechanisms.intake.start();
                    follower.followPath(toIntakeB, false);
                    mechanisms.spindexer.autoIntakeEnabled = true;
                    mechanisms.spindexer.home();
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(intakingMaxPower);
                    follower.followPath(intakingB, false);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()){
                    follower.setMaxPower(globalMaxPower);
                    mechanisms.intake.passiveIntake();
                    follower.followPath(scoreB, true);
                    mechanisms.spindexer.autoIntakeEnabled = false;
                    shotsFired = 0;
                    pathState = 9;
                }
                break;

            case 9:
                // Wait for launcher to get up to speed and launch 3 pixels
                if (!follower.isBusy()) {
                    if (launchAllState == LaunchAllState.Inactive) {
                        launchAllState = LaunchAllState.ReadyToFire;
                    } else if (launchAllState == LaunchAllState.Complete) {
                        launchAllState = LaunchAllState.Inactive;
                        pathState = 10;
                    }
                }
                break;

            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(park, false);
                    mechanisms.stopAll();
                    pathState = -1;
                }
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        mechanisms.update();
        launchAll();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Launch All State", launchAllState);
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("Spindexer Moving", mechanisms.spindexer.isMoving());
        telemetry.addData("Kicker Timer", kickerTimer.milliseconds());
        /*telemetry.addData("Hood Position", "%.3f", mechanisms.hood.getPosition());
        telemetry.addData("Hood Target", "%.3f", mechanisms.hood.getTargetPosition());
        telemetry.addData("Hood Holding", mechanisms.hood.isHoldingPosition() ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Hood Voltage", "%.3fV", mechanisms.hood.getEncoderVoltage());
        */telemetry.update();
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
                (mechanisms.spindexer.isLimitPressed() ? "PRESSED" : "Released"));
        telemetry.addLine("Auto-Intake: ENABLED by default");
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
    }

    public void launchAll(){
        switch (launchAllState){
            case Inactive:
                // Do nothing, waiting to start
                break;

            case ReadyToFire:
                // Start the kick sequence
                if(!mechanisms.spindexer.isMoving() && mechanisms.launcher.getActualRPM() > minLauncherVelocity && mechanisms.launcher.getActualRPM() < maxLauncherVelocity){
                    mechanisms.launcher.kick();
                    kickerTimer.reset();
                    launchAllState = LaunchAllState.Firing;
                }

                break;

            case Firing:
                // Wait for kick to complete
                if (kickerTimer.milliseconds() > 1000 && !mechanisms.launcher.isKicking()) {
                    shotsFired++;
                    if (shotsFired >= 3) {
                        launchAllState = LaunchAllState.Complete;
                    } else {
                        launchAllState = LaunchAllState.RotatingSpindexer;
                    }
                }
                break;
            case RotatingSpindexer:
                mechanisms.spindexer.increment();
                launchAllState = LaunchAllState.ReadyToFire;
                break;
            case Complete:
                // Do nothing, wait for path to continue
                break;
        }
    }
}