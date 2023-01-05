package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.structure.ServoClaw;
import org.firstinspires.ftc.teamcode.drive.structure.Slider;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class Consctructor_Auto extends LinearOpMode {

    private Sleeve_Detection sleeveDetection = new Sleeve_Detection();
    private OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";


    Slider sd = new Slider();

    int caz = 2;

    ServoClaw sclaw = new ServoClaw();

    int Target;

    enum State {
        TRAJECTORY_1,
        TRAJECTORY_2,
        TURN_1,
        TRAJECTORY_3,
        WAIT_1,
        TURN_2,
        DECIDE,
        IDLE
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(15, 10, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new Sleeve_Detection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });


        sd.init(hardwareMap,null);
        sclaw.init(hardwareMap,null);

        sclaw.Open();

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .forward(65)
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeLeft(12)
                .build();


        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .forward(15.5)
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .back(15)
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .strafeRight(15)
                .build();

        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .back(30)
                .build();

        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .strafeLeft(40)
                .build();

        Trajectory trajectory8A = drive.trajectoryBuilder(trajectory7.end())
                .back(25)
                .build();

        Trajectory trajectory8B = drive.trajectoryBuilder(trajectory7.end())
                .forward(25)
                .build();

        double turnAngle1 = Math.toRadians(-270);

        Pose2d newLastPose = trajectory2.end().plus(new Pose2d(0, 0, turnAngle1));

        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        double turnAngle2 = Math.toRadians(720);



        while(!isStarted()){

            if(sleeveDetection.getPosition() == Sleeve_Detection.ParkingPosition.LEFT)
            {
                caz = 1;
            }else if(sleeveDetection.getPosition() == Sleeve_Detection.ParkingPosition.CENTER)
            {
                caz = 2;
            }else caz = 3;


            telemetry.addData("caz",caz);
            telemetry.update();

        }

        waitForStart();


        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentState) {
                case TRAJECTORY_1:
                    Target = 10500;
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case TRAJECTORY_2:
                    if (!drive.isBusy()) {
                        currentState = State.TURN_1;
                        drive.followTrajectory(trajectory3);
                    }
                    break;
                case TURN_1:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_3;
                        sclaw.Closed();

                        sleep(500);

                        drive.followTrajectoryAsync(trajectory4);
                        Target = 0;
                    }
                    break;
               case TRAJECTORY_3:
                    if (!drive.isBusy()) {

                        currentState = State.WAIT_1;
                    //    currentState = State.WAIT_1;

                     //    drive.followTrajectory(trajectory5);

                        drive.followTrajectory(trajectory5);

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                      //  waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                  if(!drive.isBusy()){
                      drive.followTrajectory(trajectory6);
                      currentState = State.TURN_2;
                  }
                    break;
                case TURN_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        drive.followTrajectory(trajectory7);
                        currentState = State.DECIDE;
                    }
                    break;
                 case DECIDE:
                   if(!drive.isBusy()){
                       if(caz == 1)
                       {
                           drive.followTrajectory(trajectory8A);
                       }else if(caz == 3){
                           drive.followTrajectory(trajectory8B);
                       }

                       currentState = State.IDLE;
                   }
            /*    case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
           */
                case IDLE:
                    break;
            }
            drive.update();

            Update();

            telemetry.addData("Is slider busy",sd.sliderMotor.isBusy());
            telemetry.addData("Slider's current posituion",sd.GetSliderPosition());
            telemetry.update();

        }
    }


   public void Update(){

            sd.sliderMotor.switchToLevel(Target);

            sd.sliderMotor.SetPower(1);

            sd.sliderMotor.StateUpdate(true);

          //  sd.sliderMotor.SetPower(1);

     //      if(!sd.sliderMotor.isBusy()) sd.sliderMotor.SetPower(0);
   }



}
