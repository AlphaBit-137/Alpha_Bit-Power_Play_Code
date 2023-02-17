package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Gyroscope;

public class Centric_Drive {

    ChasisInit chasis = new ChasisInit();
    Gyroscope gyroscope = new Gyroscope();
    Gamepad chasis_gamepad;

    public void Init(HardwareMap hwmap,Gamepad chasis_gamepad)
    {
        gyroscope.Init(hwmap);
        chasis.init(hwmap);
        this.chasis_gamepad = chasis_gamepad;
    }


    double speed = 1.0;

    final double controllerDeadzone = 0.15;

    double rotatedX = 0;
    double rotatedY = 0;


    public void run() {

        gyroscope.updateOrientation();

        double x = -chasis_gamepad.left_stick_x * 1.1;
        double y = chasis_gamepad.left_stick_y ;
        double r = -chasis_gamepad.right_stick_x;

        double neededOffset = -Math.toRadians(-gyroscope.getHeading());

        rotatedX = x * Math.cos(neededOffset) - y * Math.sin(neededOffset);
        rotatedY = x * Math.sin(neededOffset) + y * Math.cos(neededOffset);

        rotatedX = addons(rotatedX);
        rotatedY = addons(rotatedY);
        r = addons(r);

        double normalizer = Math.max(Math.abs(rotatedX) + Math.abs(rotatedY) + Math.abs(r), 1.0);

        double FrontLeftPower = (rotatedY + rotatedX + r) / normalizer;
        double FrontRightPower = (rotatedY - rotatedX - r) / normalizer;
        double BackLeftPower = (rotatedY - rotatedX + r) / normalizer;
        double BackRightPower = (rotatedY + rotatedX - r) / normalizer;

        chasis.FrontLeft.setPower(FrontLeftPower);
        chasis.FrontRight.setPower(FrontRightPower);
        chasis.BackLeft.setPower(BackLeftPower);
        chasis.BackRight.setPower(BackRightPower);
    }

    private double addons(double coord) {
        if (Math.abs(coord) < controllerDeadzone) {
            coord = 0;
        }
        coord = coord * speed;

        return coord;
    }

}

