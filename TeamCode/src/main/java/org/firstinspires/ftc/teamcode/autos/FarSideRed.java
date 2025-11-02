package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.AprilTagDetector;
import org.firstinspires.ftc.teamcode.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class FarSideRed extends OpMode{
    private DECODEMechanisms mechanisms;
    private AprilTagDetector aprilTagDetector;

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
                    follower.followPath(toIntakeA1, true);
                    pathState = 1;
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(intakingA1, false);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(shootA1, false);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(toIntakeB, true);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(intakingB, false);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(shootB, false);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()){
                    follower.followPath(park, true);
                    pathState = 7;
                }
                break;

        }
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();



        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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
    }

    @Override
    public void init_loop() {
        telemetry.addLine("DO NOT RUN WITH BATTERY BELOW 13 VOLTS!!!!!");
        telemetry.addLine("IF YOU DO I WILL FIND YOU :)");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathState = 0;
    }
}
