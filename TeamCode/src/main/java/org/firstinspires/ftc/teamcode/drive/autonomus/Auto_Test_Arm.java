package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.ServoClaw;
import org.firstinspires.ftc.teamcode.drive.structure.Slider;

@Autonomous
public class Auto_Test_Arm extends LinearOpMode {

    ElapsedTime stimer = new ElapsedTime();

    Arm arm = new Arm();
    Slider lift = new Slider();
    ServoClaw sclaw = new ServoClaw();
    GetDetection camera = new GetDetection();
    SampleMecanumDrive drive;

    ElapsedTime Timer_for_arm = new ElapsedTime();

    int caz = 1;

    double x_add=0, y_add=0, angleAdd=0, armAdd=0;

    double stack_angle_add;

    double stack_x_add = 0;

    double y_pole_add = 0;

    int Loops = 3, i = 0, j = 0;

    Gamepad Null;

    double slider_ref = 210;
    double arm_ref = 410;
    double servo_position = 0.53;

    Pose2d startPose = new Pose2d( -35.5, -56, Math.toRadians(90));


    Pose2d case1 = new Pose2d(-13.442408343868342,-12.603328049212077, Math.toRadians( 180-178.2355014527972));

    Pose2d case2 = new Pose2d(-34.30768651317242,-12.541090024383394,Math.toRadians(182.30987321724987-180));

    Pose2d case3 = new Pose2d(-59.10302904448052,-10.39709841950815,Math.toRadians(185.2383304842712-180));

    TrajectorySequence case_1,case_2,case_3;

    Vector2d line = new Vector2d( -36.86593279210669,-24.608077437197178);
    Vector2d poleFirst = new Vector2d(-24.197146082134117,-12.039688813835287);
    TrajectorySequence traj;


    Pose2d[] coords_stack = new Pose2d[6];
    Pose2d[] coords_pole = new Pose2d[6];
    TrajectorySequence[] pole_traj = new TrajectorySequence[6];
    TrajectorySequence[] stack_traj = new TrajectorySequence[6];
    TrajectorySequence[] caseOne = new TrajectorySequence[6];
    TrajectorySequence[] caseTwo = new TrajectorySequence[6];
    TrajectorySequence[] caseThree = new TrajectorySequence[6];


    ElapsedTime reference_timer = new ElapsedTime();

    boolean rise_boolean = true;

    enum Paths{
        FirstCone,
        Stack_second_cone,
        Pole_second_cone,
        Park,
        idle
    }

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    Auto_lEFT.Paths paths = Auto_lEFT.Paths.FirstCone;

    @Override
    public void runOpMode() throws InterruptedException {

        TelemetryPacket packet =new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);

        sclaw.init(hardwareMap,Null,true);
        lift.init(hardwareMap,Null);
        arm.init(hardwareMap,Null);
        camera.initCamera(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectoryVelocityConstraint vc  = new TranslationalVelocityConstraint(60);

        traj = drive.trajectorySequenceBuilder(startPose)
                .lineTo(line)
                .splineToConstantHeading(poleFirst,Math.toRadians(180-92.92843015970804))
                .build();


        for(int i = 0; i <= 5; i++)
        {
            if(i == 0)
            {
                angleAdd=0;

                x_add -= 1.3;

                y_add += 1.5;

                stack_x_add = -0.8;

                stack_angle_add = 0;

                // angleAdd += 10;
                angleAdd -= 7;


                //cresc x scad y
            }else {
                //    x_add += 0.5; y_add -= 0.5; angleAdd += 1.2; stack_x_add +=1;
                //  x_add += 0.3; y_add -= 0.4; angleAdd += 1; stack_x_add +=0.8;
                //   x_add += 0.3; y_add -= 0.4; angleAdd += 1; stack_x_add +=0.8; y_pole_add -= 0;
                x_add -= 1; y_add -= 1.5 ; angleAdd -= 2; y_pole_add -= 0.5; stack_angle_add -= 1;
                //   x_add += 0.8; y_add -= 1.5 ; angleAdd += 3; y_pole_add -= 0.45; stack_angle_add -= 1;

                if(i < 2)
                {
                    stack_x_add -=0.2;
                }else stack_x_add -= 0.1;

            }

            //  coords_stack[i] = new Pose2d(53.47516145341139-0.3 - x_add,-13.069087565956906 + y_add,3.176351446923455);
            //  coords_stack[i] = new Pose2d(53.47516145341139-0.3 + stack_x_add,-13.069087565956906 + y_add,3.176351446923455);

            if(i < 2) {
                coords_stack[i] = new Pose2d(-51.57516145341139 - 0.3 + stack_x_add, -13.009087565956906 + y_add, Math.toRadians(stack_angle_add));
            }else coords_stack[i] = new Pose2d(-50.67516145341139 - 0.3 + stack_x_add, -13.009087565956906 + y_add, Math.toRadians(stack_angle_add));

            //  coords_pole[i] =  new Pose2d(35.27549268087416 + x_add,-12.547379192844328 + y_add,Math.toRadians(125.08902579940354+angleAdd));
            coords_pole[i] =  new Pose2d(-34.27549268087416 + x_add,-13.547379192844328 + y_pole_add,Math.toRadians(180-125.08902579940354+angleAdd));

            if(i == 0)
            {


                stack_traj[i] = drive.trajectorySequenceBuilder(traj.end())
                        .lineToLinearHeading(coords_stack[i])
                        .build();


                pole_traj[i] = drive.trajectorySequenceBuilder(stack_traj[i].end())
                        .lineToLinearHeading(coords_pole[i])
                        .build();

            }else{
                stack_traj[i] = drive.trajectorySequenceBuilder(pole_traj[i-1].end())
                        .lineToLinearHeading(coords_stack[i])
                        .build();

                pole_traj[i] = drive.trajectorySequenceBuilder(stack_traj[i].end())
                        .lineToLinearHeading(coords_pole[i])
                        .build();
            }


            caseOne[i] = drive.trajectorySequenceBuilder(pole_traj[i].end())
                    .lineToLinearHeading(case1)
                    .build();

            caseTwo[i] = drive.trajectorySequenceBuilder(pole_traj[i].end())
                    .lineToLinearHeading(case2)
                    .build();

            caseThree[i] = drive.trajectorySequenceBuilder(pole_traj[i].end())
                    .lineToLinearHeading(case3)
                    .build();

        }

        case_1 = drive.trajectorySequenceBuilder(pole_traj[0].end())
                .lineToLinearHeading(case1)
                .build();

        case_2 = drive.trajectorySequenceBuilder(pole_traj[0].end())
                .lineToLinearHeading(case2)
                .build();

        case_3 = drive.trajectorySequenceBuilder(pole_traj[0].end())
                .lineToLinearHeading(case3)
                .build();

        sclaw.startPos();
        sclaw.Closed();


        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        while(!opModeIsActive())
        {
            camera.Detect();
            caz = camera.getCase();
            packet.put("case",caz);
            telemetry.addData("case",caz);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();


            Timer_for_arm.reset();

        }



        waitForStart();

        if(isStopRequested())return;

      //  drive.followTrajectorySequenceAsync(traj);

        while(!isStopRequested() && opModeIsActive())
        {

            switch (paths) {
                case FirstCone:

                    if (rise_boolean) {
                        rise_boolean = !rise_boolean;
                        reference_timer.reset();
                    }

                    if (reference_timer.seconds() > 0.5){
                        sclaw.centrationServo.setPosition(0.33);
                        sclaw.rotationServo.setPosition(0.75);
                        lift.setReference(700);
                        arm.setReference(1850);
                    }

                    if(Timer_for_arm.seconds() > 2)
                    {
                        sleep(300);
                        sclaw.Open();
                        sleep(100);


                        //drive.followTrajectorySequenceAsync(stack_traj[i]);
                        i++;
                        rise_boolean = !rise_boolean;
                        paths = Auto_lEFT.Paths.Stack_second_cone;


                        Timer_for_arm.reset();
                    }

                    break;
                case Stack_second_cone:

                    if(rise_boolean)
                    {
                        rise_boolean = !rise_boolean;
                        reference_timer.reset();
                    }

                    if(reference_timer.seconds() > 0.7)
                    {

                        sclaw.startPos();
                        //for stack
                        sclaw.centrationServo.setPosition(0.43);

                        /**
                         lift.setReference(320);
                         arm.setReference(280);
                         */

                        lift.setReference(slider_ref);
                        arm.setReference(arm_ref);

                    }

                    if(Timer_for_arm.seconds() > 2)
                    {
                        sclaw.Closed();
                        lift.setReference(750);

                        sleep(500);

                        sliderRun();
                        sleep(500);
                        sclaw.conePose();
                        sclaw.centrationServo.setPosition(0.6);

                       // drive.followTrajectorySequenceAsync(pole_traj[j]);


                        j++;

                        slider_ref -= 15;
                        arm_ref -= 30;
                        servo_position -= 0.01;

                        rise_boolean = !rise_boolean;
                        reference_timer.reset();

                        Timer_for_arm.reset();


                        // paths = Paths.idle;
                        paths = Auto_lEFT.Paths.Pole_second_cone;

                    }

                    break;

                case Pole_second_cone:

                    if(rise_boolean)
                    {
                        rise_boolean = false;
                        reference_timer.reset();
                    }
                    if(reference_timer.seconds() > 0.2)
                    {
                        arm.setReference(2200);
                    }

                    if(Timer_for_arm.seconds() > 2) {

                        sclaw.Open();

                        sleep(600);

                        Loops--;

                        if(Loops > 0) {
                          //  drive.followTrajectorySequenceAsync(stack_traj[i]);
                            i++;
                            armAdd-=50;
                            slider_ref -= 110;
                            paths = Auto_lEFT.Paths.Stack_second_cone;

                            Timer_for_arm.reset();

                        }else{
                            lift.setReference(0);
                            arm.setReference(0);
                            sclaw.Open();
                            // sclaw.stackPose();
                            sclaw.stackPose();
                            sclaw.pickedUpCone();
                            //sclaw.stackPose();

                            if(caz == 1) {
                            //    drive.followTrajectorySequenceAsync(caseOne[i - 1]);
                            }else if(caz == 2)
                            {
                              //  drive.followTrajectorySequenceAsync(caseTwo[i-1]);
                            }else // drive.followTrajectorySequenceAsync(caseThree[i-1]);

                            Timer_for_arm.reset();

                            paths = Auto_lEFT.Paths.idle;
                        }

                        Timer_for_arm.reset();


                    }
                    //  }


                    break;

                case Park:

                    sclaw.Open();
                    sclaw.stackPose();
                    arm.setReference(0);
                    lift.setReference(0);

                    if(caz==1) {
                        drive.followTrajectorySequenceAsync(case_1);
                    }
                    else if(caz==2) {
                        drive.followTrajectorySequenceAsync(case_2);
                    }
                    else {
                        drive.followTrajectorySequenceAsync(case_3);
                    }

                    if(!drive.isBusy())
                    {
                        paths = Auto_lEFT.Paths.idle;
                    }

                    break;
                case idle:
                    break;
            }

            lift.autoUpdate();
            arm.autoUpdate();
            drive.update();

            telemetry.addData("i",i);
            telemetry.addData("j",j);

            telemetry.update();


            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }

    }

    public void sliderRun()
    {
        stimer.reset();
        while(stimer.seconds() < 0.1)
        {
            lift.setReference(550);
            lift.autoUpdate();
        }
    }



}
