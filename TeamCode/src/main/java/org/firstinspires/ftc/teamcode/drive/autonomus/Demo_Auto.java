package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.structure.ServoClaw;


@Autonomous
public class Demo_Auto extends LinearOpMode{

    SampleMecanumDrive drive;

    Trajectory trajectory;
    Trajectory trajectory2;
    Trajectory LidPid;

    ServoClaw sc = new ServoClaw();

    @Override
    public void runOpMode() throws InterruptedException {

        sc.init(hardwareMap,null);
        drive = new SampleMecanumDrive(hardwareMap);

        trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(160)
                .build();


        //40 = 90 degrees

        trajectory2 = drive.trajectoryBuilder(trajectory.end())
                .strafeLeft(35)
                .build();

        LidPid = drive.trajectoryBuilder(trajectory2.end())
                .forward(100)
                .build();


        waitForStart();

        if(isStopRequested())return;


        drive.followTrajectory(trajectory);

        drive.followTrajectory(trajectory2);

        drive.followTrajectoryAsync(LidPid);

    }
}
