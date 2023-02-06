package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.structure.ServoClaw;


@Autonomous
public class Demo_Auto extends LinearOpMode{

    SampleMecanumDrive drive;

    ServoClaw sc = new ServoClaw();

    final double angleAdd =/*6.229321263870247*/10;

    final int parteAuto = 1; //1 pentru dreapta, -1 pentru stanga
     int schimbUnghi;
     boolean reverse, r2;
     int k;

    Pose2d pole;
    Pose2d stack;
    Pose2d lilbit;
    Pose2d startPose;


    @Override
    public void runOpMode() throws InterruptedException {

        startPose = new Pose2d(parteAuto * 40, -65, 1.6);
        pole = new Pose2d(parteAuto * 30, -9, Math.toRadians(k * (schimbUnghi - 120) + (angleAdd * parteAuto)));
        stack = new Pose2d(parteAuto * 65, -14, Math.toRadians(k * (schimbUnghi - 180) + (angleAdd * parteAuto)));
        lilbit = new Pose2d(parteAuto * 45, -11, Math.toRadians(k * (schimbUnghi - 160) + (angleAdd * parteAuto)));

        if (parteAuto < 0) {
            reverse = false;
            r2 = reverse;
            schimbUnghi = 180;
            k = 1;
        } else {
            reverse = true;
            r2 = !reverse;
            schimbUnghi = 0;
            k = -1;
        }

        sc.init(hardwareMap,null);
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj;

        traj = drive.trajectorySequenceBuilder(startPose)

                .lineTo(new Vector2d(parteAuto * 32, -15))
                .splineToConstantHeading(new Vector2d(parteAuto * 24, -10), Math.toRadians(k * (schimbUnghi - 0) ))
                .waitSeconds(0.3)
              //  .lineTo(new Vector2d(33, -12))
                .setReversed(reverse)
               // .splineToLinearHeading(new Pose2d(60, -12, Math.toRadians(180)), Math.toRadians(0))
                .lineToLinearHeading(stack)

                .waitSeconds(0.3)

                //.lineToLinearHeading(lilbit)
                .lineToLinearHeading(pole)

                //loop

                .waitSeconds(0.3)
              //  .setReversed(!reverse)

                .lineToLinearHeading(stack)

                /*.waitSeconds(0.3)

                //.lineToLinearHeading(lilbit)
                .lineToLinearHeading(pole)

                //loop
                .waitSeconds(0.3)

                .lineToLinearHeading(stack)

                .waitSeconds(0.3)

                //.lineToLinearHeading(lilbit)
                .lineToLinearHeading(pole)

                //loop

                .waitSeconds(0.3)

                .lineToLinearHeading(stack)

                .waitSeconds(0.3)

                //.lineToLinearHeading(lilbit)
                .lineToLinearHeading(pole)

                //loop

                .waitSeconds(0.3)

                .lineToLinearHeading(stack)

                .waitSeconds(0.3)

                //.lineToLinearHeading(lilbit)
                .lineToLinearHeading(pole)

                //loop

                .waitSeconds(0.3)

                //end of loop

                .setReversed(!r2)

                .splineTo(new Vector2d(parteAuto * 36, -30), Math.toRadians(-90 - angleAdd))

                .lineTo(new Vector2d(parteAuto * 36, -36))

                .waitSeconds(0.3)
*/

                .build();

        waitForStart();

        drive.followTrajectorySequence(traj);

        if(isStopRequested())return;



    }
}
