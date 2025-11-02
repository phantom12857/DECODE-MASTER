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
    private final ElapsedTime timer = new ElapsedTime();
    private double globalMaxPower = 0.8;
    private double intakingMaxPower = 0.2;
    private boolean launch;
    private int launchCounts = 0;
    public enum LaunchAllState {
        Inactive,
        Launch1,
        Rotate2,
        Launch2,
        Rotate3,
        Launch3,
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
    private final Pose scanAndScorePLPose = new Pose(48, 98, Math.toRadians(122));
    private final Pose a3IntakePose = new Pose(52, 84, Math.toRadians(180));
    private final Pose a3EndPose = new Pose(25,84, Math.toRadians(180));
    private final Pose scorePose = new Pose(48,98, Math.toRadians(122 ));
    private final Pose a2IntakePose = new Pose(45, 59, Math.toRadians(180));
    private final Pose a2EndPose = new Pose(20,59, Math.toRadians(180));
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
                    mechanisms.setLauncherRPM(3250);

                    mechanisms.fireKicker();
                    new Thread(() -> {
                        try {
                            Thread.sleep(500);
                            mechanisms.retractKicker();
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }).start();
                    //if(launchCounts == 3){
                        pathState = 1;
                    //}
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(toIntakeA3, false);
                    mechanisms.startIntake();
                    mechanisms.reverseIntake();
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(intakingMaxPower);
                    follower.followPath(intakingA3, false);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(globalMaxPower);
                    follower.followPath(scoreA3, true);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(toIntakeA2, false);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(intakingMaxPower);
                    follower.followPath(intakingA2, false);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()){
                    follower.setMaxPower(globalMaxPower);
                    follower.followPath(scoreA2, true);
                    pathState = 7;
                }
                break;

            case 7:
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
        kickerControls();
        launchAll();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Launch All State", launchAllState);
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
    }

    @Override
    public void stop(){
        mechanisms.stopAllMotors();
    }

    private void kickerControls(){
        if (launch == true){
            mechanisms.fireKicker();

        }
    }
    public void launchAll(){
        switch (launchAllState){
            case Inactive:
                if(launch == true){
                    launchAllState = LaunchAllState.Launch1;
                }
                break;
            case Launch1:
                launch = true;
                if(launch == false){
                    launchAllState = LaunchAllState.Rotate2;
                }
                break;

            case Rotate2:
                mechanisms.manualAdvanceSpindexer();
                if(!mechanisms.isSpindexerMoving()){
                    launchAllState = LaunchAllState.Launch2;
                }
                break;

            case Launch2:
                launch = true;
                if(launch == false){
                    launchAllState = LaunchAllState.Rotate3;
                }
                break;

            case Rotate3:
                mechanisms.manualAdvanceSpindexer();
                if(!mechanisms.isSpindexerMoving()){
                    launchAllState = LaunchAllState.Launch3;
                }
                break;

            case Launch3:
                launch = true;
                if(launch == false){
                    launchAllState = LaunchAllState.Inactive;
                }
                break;

        }
    }


}
