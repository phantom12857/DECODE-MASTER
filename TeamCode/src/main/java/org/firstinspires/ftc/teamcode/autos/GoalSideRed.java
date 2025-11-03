package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.tests.AprilTagDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class GoalSideRed extends OpMode{
    private DECODEMechanisms mechanisms;
    private AprilTagDetector aprilTagDetector;
    private final ElapsedTime kickerTimer = new ElapsedTime();
    private double globalMaxPower = 0.8;
    private double intakingMaxPower = 0.3;
    private int shotsFired = 0;

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
    private final Pose startPose = new Pose(100.43, 134.75, Math.toRadians(90));
    private final Pose scanAndScorePLPose = new Pose(92, 90, Math.toRadians(33));
    private final Pose a3IntakePose = new Pose(95, 84, Math.toRadians(180));
    private final Pose a3EndPose = new Pose(120,84, Math.toRadians(180));
    private final Pose scorePose = new Pose(105,95, Math.toRadians(30));
    private final Pose a2IntakePose = new Pose(95, 59, Math.toRadians(180));
    private final Pose a2EndPose = new Pose(120,59, Math.toRadians(180));
    private final Pose parkPose = new Pose(105, 70, Math.toRadians(180));

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
                .addPath(new BezierLine(a2EndPose, scorePose))
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
                if (!follower.isBusy()) {
                    follower.setMaxPower(globalMaxPower);
                    follower.followPath(scanAndScorePL, true);
                    mechanisms.launcher.setRPM(3250);
                    mechanisms.intake.start();
                    mechanisms.intake.reverse();
                    shotsFired = 0;
                    pathState = 1;
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (launchAllState == LaunchAllState.Inactive) {
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
                if (!follower.isBusy()) {
                    if (launchAllState == LaunchAllState.Inactive) {
                        shotsFired = 0;
                        launchAllState = LaunchAllState.ReadyToFire;
                    } else if (launchAllState == LaunchAllState.Complete) {
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
                break;

            case ReadyToFire:
                mechanisms.launcher.kick();
                launchAllState = LaunchAllState.Firing;
                kickerTimer.reset();
                break;

            case Firing:
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
                break;
        }
    }
}