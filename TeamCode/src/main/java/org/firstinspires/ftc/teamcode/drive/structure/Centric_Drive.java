package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Gyroscope;

public class Centric_Drive {

    ChasisInit chasis;
    Gyroscope gyroscope;

    public void Init(HardwareMap hwmap)
    {
        gyroscope.Init(hwmap);
        chasis.init(hwmap);
    }


    double speed = 1.0;
    final double controllerDeadzone = 0.15;

    double rotatedX = 0;
    double rotatedY = 0;


    public void run(double left_stick_x,double right_stick_y, double left_stick_y) {

        gyroscope.updateOrientation();

        double x = left_stick_x;
        double y = -right_stick_y;
        double r = -left_stick_y;

        double neededOffset = -Math.toRadians(gyroscope.getHeading());

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

