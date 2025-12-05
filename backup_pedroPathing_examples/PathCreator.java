package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.BezierLine;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;

/**
 * The PathCreator class centralizes the definitions of all autonomous paths and poses.
 * This prevents code duplication and makes path adjustments easier to manage.
 */
public class PathCreator {

    // ==================================================
    // P O S E S
    // ==================================================

    // Blue Alliance Poses
    public static final Pose startPoseGoalSideBlue = new Pose(43.5, 134.75, Math.toRadians(90));
    public static final Pose startPoseFarSideBlue = new Pose(51.75, 8.625, Math.toRadians(90));

    // Red Alliance Poses
    public static final Pose startPoseGoalSideRed = new Pose(96.5, 134.75, Math.toRadians(90));
    public static final Pose startPoseFarSideRed = new Pose(88.25, 8.625, Math.toRadians(90));

    // Shared Poses
    public static final Pose scorePose = new Pose(48, 98, Math.toRadians(130));
    public static final Pose intakePose1 = new Pose(52, 84, Math.toRadians(180));
    public static final Pose intakePose2 = new Pose(45, 61, Math.toRadians(180));
    public static final Pose parkPose = new Pose(35, 70, Math.toRadians(180));

    // ==================================================
    // P A T H   C R E A T I O N
    // ==================================================

    /**
     * Creates a path from a starting pose to the main scoring position.
     */
    public static PathChain createPathToGoal(Follower follower, Pose start) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, scorePose))
                .setLinearHeadingInterpolation(start.getHeading(), scorePose.getHeading())
                .build();
    }

    /**
     * Creates a path from the scoring position to an intake position.
     */
    public static PathChain createPathToIntake(Follower follower, Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    /**
     * Creates a path from the scoring position to the parking position.
     */
    public static PathChain createPathToPark(Follower follower, Pose start) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, parkPose))
                .setLinearHeadingInterpolation(start.getHeading(), parkPose.getHeading())
                .build();
    }
}
