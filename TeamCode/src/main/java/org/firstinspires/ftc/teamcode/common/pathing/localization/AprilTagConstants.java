package org.firstinspires.ftc.teamcode.common.pathing.localization;


import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

public class AprilTagConstants {
    public static final double Y_OFFSET = MathUtils.mmToInches(215);
    public static final double X_OFFSET = 0;
    public static final Pose CAMERA_POSE= new Pose(X_OFFSET, Y_OFFSET, 0);

    public static final Pose BLUE_BACKDROP_POSITION = CenterstageConstants.BLUE_BACKDROP_POSITION;
    public static final Pose RED_BACKDROP_POSITION = CenterstageConstants.RED_BACKDROP_POSITION;
    public static Pose pose;

    public static Pose convertBlueBackdropPoseToGlobal(Pose pipeline) {
        Pose offset = new Pose();
        pose = new Pose();


        offset.setX(CAMERA_POSE.getX() * Math.cos(pipeline.getHeading()) - CAMERA_POSE.getY() * Math.sin(pipeline.getHeading()));
        offset.setY(CAMERA_POSE.getX() * Math.sin(pipeline.getHeading()) + CAMERA_POSE.getY() * Math.cos(pipeline.getHeading()));

        pose.add(BLUE_BACKDROP_POSITION);
        pose.subtract(pipeline);
        pose.subtract(offset);
        return pose;
    }

    public static Pose convertRedBackdropPoseToGlobal(Pose pipeline) {
        Pose offset = new Pose();
        pose = new Pose();

        offset.setX(CAMERA_POSE.getX() * Math.cos(pipeline.getHeading()) - CAMERA_POSE.getY() * Math.sin(pipeline.getHeading()));
        offset.setY(CAMERA_POSE.getX() * Math.sin(pipeline.getHeading()) + CAMERA_POSE.getY() * Math.cos(pipeline.getHeading()));

        pose.add(RED_BACKDROP_POSITION);
        pose.subtract(pipeline);
        pose.subtract(offset);
        return pose;
    }
}
