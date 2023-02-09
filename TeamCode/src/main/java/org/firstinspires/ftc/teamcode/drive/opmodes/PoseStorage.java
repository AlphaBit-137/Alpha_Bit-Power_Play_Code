package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {

    public static Pose2d currentPose = new Pose2d();

    //You have to use this command to always save the robot's position
    //PoseStorage.currentPose = drive.getPoseEstimate();

}
