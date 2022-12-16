package org.firstinspires.ftc.teamcode.drive.structure;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

public class Robot_Centric_Drive {

    Gamepad Robot_Gamepad;
    HardwareMap hwmap;

    SampleMecanumDrive drive;

    public void init(HardwareMap hwmap , Gamepad Robot_Gamepad){
        drive = new SampleMecanumDrive(hwmap);
        this.Robot_Gamepad = Robot_Gamepad;
    }

    public void run()
    {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -Robot_Gamepad.left_stick_y,
                        -Robot_Gamepad.left_stick_x,
                        -Robot_Gamepad.right_stick_x
                )
        );

        drive.update();

    }


}
