package com.example.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        final double angleAdd =/*6.229321263870247*/0;

        final int parteAuto = 1; //1 pentru dreapta, -1 pentru stanga
        final int schimbUnghi;
        final boolean reverse, r2;
        final int k;

        if (parteAuto == -1) {
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

        Pose2d pole = new Pose2d(parteAuto * 30, -9, Math.toRadians(k * (schimbUnghi - 120) + angleAdd));
        Pose2d stack = new Pose2d(parteAuto * 65, -14, Math.toRadians(k * (schimbUnghi - 180) + angleAdd));
        Pose2d lilbit = new Pose2d(parteAuto * 45, -11, Math.toRadians(k * (schimbUnghi - 160) + angleAdd));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(14.17, 17.32)
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 13.94)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(parteAuto * 40, -65, 1.6))
                                .lineTo(new Vector2d(parteAuto * 32, -15))
                                .splineToConstantHeading(new Vector2d(parteAuto * 24, -10), Math.toRadians(k * (schimbUnghi - 0) + angleAdd))
                                .waitSeconds(0.3)
                                //.lineTo(new Vector2d(33, -12))
                                .setReversed(reverse)
                                //.splineToLinearHeading(new Pose2d(60, -12, Math.toRadians(180)), Math.toRadians(0))
                                .lineToLinearHeading(stack)

                                .waitSeconds(0.3)

                                //.lineToLinearHeading(lilbit)
                                .lineToLinearHeading(pole)

                                //loop

                                .waitSeconds(0.3)
                                //.setReversed(!reverse)

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


                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}