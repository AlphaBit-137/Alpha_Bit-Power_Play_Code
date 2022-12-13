package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.structure.Centric_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.ServoClaw;
import org.firstinspires.ftc.teamcode.drive.structure.Slider;

@TeleOp
public class First extends LinearOpMode {


    Centric_Drive CDrive = new Centric_Drive();
    Slider slider = new Slider(gamepad2);
    ServoClaw claw = new ServoClaw();


    @Override
    public void runOpMode() throws InterruptedException {

        CDrive.Init(hardwareMap);
        slider.init(hardwareMap);
        claw.init(hardwareMap);



        waitForStart();

        while(opModeIsActive()){

        }
    }



}
