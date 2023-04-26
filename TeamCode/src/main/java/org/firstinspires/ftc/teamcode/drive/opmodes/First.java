package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Gyro_Save;
import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.Centric_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.Robot_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.ServoClaw;
import org.firstinspires.ftc.teamcode.drive.structure.Servo_Arm;
import org.firstinspires.ftc.teamcode.drive.structure.Slider;

@TeleOp
public class First extends LinearOpMode {


    Robot_Drive RD = new Robot_Drive();
    Slider slider = new Slider();
    Arm arm = new Arm();

    double loopTime;

    enum drivingCase{
       RobotCentric,
       FieldCentric
    }

    drivingCase Cases = drivingCase.RobotCentric;

    @Override
    public void runOpMode() throws InterruptedException {

        RD.Init(hardwareMap,gamepad1);

        slider.init(hardwareMap,gamepad1);

        arm.init(hardwareMap,gamepad1);

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(5);
        PhotonCore.enable();

        waitForStart();

        while(opModeIsActive()){

            arm.update();
            slider.update();
            arm.update();
            RD.run();

            double loopT = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loopT - loopTime));
            telemetry.addData("ArmPosition",arm.getArmPos());

            telemetry.addData("sliderEnc",slider.GetSliderPosition());

            loopTime = loopT;

            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }
    }



}
