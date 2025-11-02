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
public class FarSideBlue extends OpMode{
    private DECODEMechanisms mechanisms;
    private AprilTagDetector aprilTagDetector;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private double globalMaxPower = 0.8;
    private double intakingMaxPower = 0.3;

    // ========== Poses ==========
    private final Pose startPose = new Pose(51.75, 8.625, Math.toRadians(90));
    private final Pose a1IntakePose = new Pose(45, 33, Math.toRadians(180));
    private final Pose a1EndPose = new Pose(15, 33, Math.toRadians(180));
    private final Pose scorePose = new Pose(51.75,14, Math.toRadians(102));
    private final Pose bIntakePose = new Pose(20,25, Math.toRadians(218));
    private final Pose bIntakeControlPose = new Pose(40, 25, Math.toRadians(225));
    private final Pose bEndPose = new Pose(14,13, Math.toRadians(218));
    private final Pose parkPose = new Pose(44, 30, Math.toRadians(90));

    // ==========  Paths  ==========
    private PathChain shootPL, toIntakeA1, intakingA1, shootA1, toIntakeB, intakingB, shootB, park;

    public void buildPaths(){
        shootPL = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        toIntakeA1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, a1IntakePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), a1IntakePose.getHeading())
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
                    follower.followPath(shootPL, true);
                    mechanisms.setLauncherRPM(4750);
                    mechanisms.startIntake();
                    mechanisms.reverseIntake();
                    pathState = 1;
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(toIntakeA1, false);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(intakingMaxPower);
                    follower.followPath(intakingA1, false);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(globalMaxPower);
                    follower.followPath(shootA1, true);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(toIntakeB, false);
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
                if (!follower.isBusy()){
                    follower.setMaxPower(.5);
                    follower.followPath(shootB, true);
                    pathState = 7;
                }
                break;

            case 7:
                if(!follower.isBusy()){
                    follower.followPath(park);
                    mechanisms.setLauncherRPM(0);
                    mechanisms.stopIntake();
                    pathState = -1;
                }

        }
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        mechanisms.updateAllSystems();


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

        mechanisms = new DECODEMechanisms(hardwareMap);
        aprilTagDetector = new AprilTagDetector(hardwareMap);


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
