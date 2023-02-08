package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.structure.Centric_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.ChasisInit;

@TeleOp
public class First_Chasis_Tests extends LinearOpMode {


    ChasisInit cs = new ChasisInit();
    Centric_Drive CDrive = new Centric_Drive();


    SampleMecanumDrive drive;


    double Xposition;
    double Yposition;
    double Heading;

    boolean toggle = false;

    Pose2d position;

    Trajectory traj;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        CDrive.Init(hardwareMap,gamepad1);
        cs.init(hardwareMap);

        position = new Pose2d(Xposition,Yposition,Heading);

        traj = drive.trajectoryBuilder(new Pose2d())
            //    .splineTo(position)
                .build();

        waitForStart();

        while (opModeIsActive())
        {
            CDrive.run();

         if(gamepad1.a)
         {
             if(toggle)
             {
                 drive.followTrajectory(traj);
             }

         }else toggle = true;

        }

    }

}
