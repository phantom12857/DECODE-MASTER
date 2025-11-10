
package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class GoalSideBlue extends OpMode{
    private DECODEMechanisms mechanisms;
    private AprilTagDetector aprilTagDetector;
    private final ElapsedTime kickerTimer = new ElapsedTime();
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private double globalMaxPower = 0.9;
    private double intakingMaxPower = .35;
    private int shotsFired = 0;
    private double minLauncherVelocity = 3375;
    private double maxLauncherVelocity = 3400;

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
    private final Pose startPose = new Pose(43.5, 134.75, Math.toRadians(90));
    private final Pose scanAndScorePLPose = new Pose(48, 98, Math.toRadians(130));
    private final Pose a3IntakePose = new Pose(52, 84, Math.toRadians(180));
    private final Pose a3EndPose = new Pose(24,84, Math.toRadians(180));
    private final Pose scorePose = new Pose(48,98, Math.toRadians(130));
    private final Pose a2IntakePose = new Pose(49, 64, Math.toRadians(180));
    private final Pose a2EndPose = new Pose(18,64, Math.toRadians(180));
    private final Pose a2ControlPose = new Pose(60, 60, Math.toRadians(0));
    private final Pose parkPose = new Pose(35, 94, Math.toRadians(180));

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
                    mechanisms.spindexer.autoIntakeEnabled = false;
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
                    follower.setMaxPower(.5);
                    mechanisms.spindexer.setBallsLoaded(0);
                    mechanisms.spindexer.home();
                    mechanisms.spindexer.autoIntakeEnabled = true;
                    mechanisms.intake.start();
                    follower.followPath(toIntakeA3, false);
                    intakeTimer.reset();
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(intakingMaxPower);
                    follower.followPath(intakingA3, false);
                    if(intakeTimer.seconds() > 5){
                        pathState = 4;
                    }
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(globalMaxPower);
                    mechanisms.spindexer.autoIntakeEnabled = false;
                    follower.followPath(scoreA3, true);
                    shotsFired = 0;
                    pathState = 5;
                }
                break;

            case 5:
                // Wait for launcher to get up to speed and launch 3 pixels
                mechanisms.intake.passiveIntake();
                if (!follower.isBusy()) {
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
                    follower.followPath(toIntakeA2, false);
                    mechanisms.spindexer.autoIntakeEnabled = true;
                    mechanisms.spindexer.home();
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
                    mechanisms.intake.passiveIntake();
                    follower.followPath(scoreA2, true);
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
        telemetry.addData("Launcher RPM", mechanisms.launcher.getActualRPM());
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