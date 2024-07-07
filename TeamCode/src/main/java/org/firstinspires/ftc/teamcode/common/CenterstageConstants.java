package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.teamcode.common.pathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Path;

public class CenterstageConstants {

    public static double[] stackHeights = {0.5, 1, 1.5, 2, 2.5};
    public static Pose redDrivetrainScoringPosition= new Pose(0, 0, 0);
    public static Pose blueDrivetrainScoringPosition= new Pose(0, 0, 0);

    public static Pose redStackIntakingPosition= new Pose(0, 0, 0);
    public static Pose blueStackIntakingPosition= new Pose(0, 0, 0);

    public static final Pose BLUE_BACKDROP_POSITION = new Pose(-35.5, 61.25, 0);
    public static final Pose RED_BACKDROP_POSITION = new Pose(35.5, 61.25, 0);
    public static Location SIDE = Location.FAR;
    /**
     * Match constants.
     */
    public static boolean IS_AUTO = true;
    public static Location ALLIANCE = Location.RED;
    public static Location RANDOMIZATION = Location.LEFT;
    public static Location PRELOAD = Location.LEFT;
    public static Location ROUTE = Location.STAGEDOOR;
    public static Path getPath(int zone, Path left, Path middle, Path right){
        if (zone==1) {
            RANDOMIZATION=Location.LEFT;
            return left;
        }
        else if (zone==2){
            RANDOMIZATION=Location.CENTER;
            return middle;
        }
        else {
            RANDOMIZATION=Location.RIGHT;
            return right;
        }
    }
}

