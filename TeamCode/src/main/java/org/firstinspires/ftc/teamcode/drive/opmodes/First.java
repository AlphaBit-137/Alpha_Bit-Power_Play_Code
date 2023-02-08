package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.Centric_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.MidWay_Servos;
import org.firstinspires.ftc.teamcode.drive.structure.Robot_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.ServoClaw;
import org.firstinspires.ftc.teamcode.drive.structure.Slider;

@TeleOp
public class First extends LinearOpMode {


    Centric_Drive CDrive = new Centric_Drive();
    Robot_Drive RD = new Robot_Drive();
    ServoClaw claw = new ServoClaw();
    Slider slider = new Slider();
    Arm arm = new Arm();
    MidWay_Servos ms = new MidWay_Servos();

    double loopTime;

    enum drivingCase{
       RobotCentric,
       FieldCentric
    }

    drivingCase Cases = drivingCase.FieldCentric;

    @Override
    public void runOpMode() throws InterruptedException {

        CDrive.Init(hardwareMap,gamepad1);

        arm.init(hardwareMap,gamepad1);

        RD.Init(hardwareMap,gamepad1);

        slider.init(hardwareMap,gamepad1);

        claw.init(hardwareMap,gamepad1);

        ms.init(hardwareMap,gamepad1);


        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(5);
        PhotonCore.enable();

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.share)
            {
                Cases = drivingCase.FieldCentric;
            }else if(gamepad1.options)
            {
                Cases = drivingCase.RobotCentric;
            }

            updateDriving();

            slider.update();
            claw.run();
            arm.update();
            ms.run();

            telemetry.addData("Slider",slider.GetSliderPosition());
            telemetry.addData("ClawS1",claw.servo1.getPosition());
            telemetry.addData("arm",arm.getArmPos());
            telemetry.addData("steady",slider.checkSteady());
            telemetry.addData("armSteady",arm.checkSteady());
            telemetry.addData("sliderPower",slider.getSliderPower());
            telemetry.addData("armPower",arm.getArmPower());

            telemetry.addData("Ref",slider.Reference);

            telemetry.addData("slider ref",slider.Reference);
            telemetry.addData("arm ref",arm.Reeference);

            telemetry.addData("red",claw.rr());

            double loopT = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loopT - loopTime));
            loopTime = loopT;

            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }
    }

    public void updateDriving()
    {
        switch(Cases)
        {
            case FieldCentric:
                CDrive.run();
                break;
            case RobotCentric:
                RD.run();
                break;
        }
    }

}
