package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.Robot_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.ServoClaw;
import org.firstinspires.ftc.teamcode.drive.structure.Slider;

@TeleOp
public class First extends LinearOpMode {


  //  Centric_Drive CDrive = new Centric_Drive();
    Robot_Drive RD = new Robot_Drive();
    ServoClaw claw = new ServoClaw();
    Slider slider = new Slider();
    Arm arm = new Arm();


    @Override
    public void runOpMode() throws InterruptedException {

       // CDrive.Init(hardwareMap);

        arm.init(hardwareMap,gamepad2);

        RD.Init(hardwareMap,gamepad1);

        slider.init(hardwareMap,gamepad2);

        claw.init(hardwareMap,gamepad2);

        waitForStart();

        while(opModeIsActive()){

         //   CDrive.run(gamepad1.left_stick_x,gamepad1.right_stick_y,gamepad1.left_stick_y);
            RD.run();
            slider.update();
            claw.run();
            arm.update();

            telemetry.addData("Slider",slider.GetSliderPosition());
            telemetry.addData("ClawS1",claw.servo1.getPosition());
            telemetry.addData("Slider",slider.sliderMotor.isBusy());



            telemetry.update();
        }
    }



}
