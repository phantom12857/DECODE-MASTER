package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.tests.AprilTagDetector;
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
public class FarSideRed extends OpMode{
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
    private final Pose startPose = new Pose(96, 9, Math.toRadians(90));
    private final Pose a1IntakePose = new Pose(95, 36, Math.toRadians(0));
    private final Pose a1EndPose = new Pose(120, 36, Math.toRadians(0));
    private final Pose scorePose = new Pose(96,9, Math.toRadians(12));
    private final Pose bIntakePose = new Pose(120,10, Math.toRadians(0));
    private final Pose bIntakeControlPose = new Pose(100, 20, Math.toRadians(0));
    private final Pose bEndPose = new Pose(130,9, Math.toRadians(0));
    private final Pose parkPose = new Pose(96, 30, Math.toRadians(90));

    // ==========  Paths  ==========
    private PathChain toIntakeA1, intakingA1, shootA1, toIntakeB, intakingB, shootB, park;

    public void buildPaths(){
        toIntakeA1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, a1IntakePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), a1IntakePose.getHeading())
                .build();

        intakingA1 = follower.pathBuilder()
                .addPath(new BezierLine(a1IntakePose, a1EndPose))
                .setLinearHeadingInterpolation(a1IntakePose.getHeading(), a1EndPose.getHeading())
                .build();

        shootA1 = follower.pathBuilder()
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

        shootB = follower.pathBuilder()
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
                if (!follower.isBusy()) {
                    follower.setMaxPower(globalMaxPower);
                    follower.followPath(toIntakeA1, true);
                    mechanisms.launcher.setRPM(4750);
                    mechanisms.intake.start();
                    mechanisms.intake.reverse();
                    pathState = 1;
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(intakingMaxPower);
                    follower.followPath(intakingA1, false);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(globalMaxPower);
                    follower.followPath(shootA1, false);
                    shotsFired = 0;
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    if (launchAllState == LaunchAllState.Inactive) {
                        launchAllState = LaunchAllState.ReadyToFire;
                    } else if (launchAllState == LaunchAllState.Complete) {
                        launchAllState = LaunchAllState.Inactive;
                        pathState = 4;
                    }
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(toIntakeB, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(intakingMaxPower);
                    follower.followPath(intakingB, false);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(.5);
                    follower.followPath(shootB, false);
                    shotsFired = 0;
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    if (launchAllState == LaunchAllState.Inactive) {
                        launchAllState = LaunchAllState.ReadyToFire;
                    } else if (launchAllState == LaunchAllState.Complete) {
                        launchAllState = LaunchAllState.Inactive;
                        pathState = 8;
                    }
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(park, true);
                    mechanisms.launcher.stop();
                    mechanisms.intake.stop();
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