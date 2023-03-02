package org.firstinspires.ftc.teamcode.drive.Testers;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.structure.ServoClaw;

@TeleOp
@Config
public class ServosTester extends LinearOpMode {

    ServoClaw sclaw = new ServoClaw();

    public static double centr = 0;
    public static double rott = 0;
    public static double claw = 0;

    @Override
    public void runOpMode() throws InterruptedException {

           sclaw.init(hardwareMap,gamepad1,false);

           waitForStart();

           while (opModeIsActive())
           {
               sclaw.centrationServo.setPosition(centr);
               sclaw.rotationServo.setPosition(rott);
               sclaw.servo1.setPosition(claw);
           }

    }
}
