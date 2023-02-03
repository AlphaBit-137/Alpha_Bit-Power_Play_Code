package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot_Drive {

    Gamepad gamepad;

    ChasisInit csint = new ChasisInit();

    public boolean Chose;
    public boolean Chose2;

    public double Limit = 0.3;

    public void Init(HardwareMap hwmap, Gamepad gamepad)
    {
        this.gamepad = gamepad;
        csint.init(hwmap);
    }

    double controllerDeadzone = 0.25;

    public void run()
    {
        double y = -gamepad.left_stick_y; // Remember, this is reversed!
        double x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;

        y = addons(y);
        x = addons(x);
        rx = addons(rx);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        MS(backLeftPower, frontRightPower, frontLeftPower, backRightPower);

        if(gamepad.right_bumper) {
            if(Chose)Limit+=0.1;
            if(Limit>1)Limit=1;
            Chose = false;
        } else Chose=true;

        if(gamepad.left_bumper) {
            if(Chose2)Limit-=0.1;
            if(Limit<0.1)Limit=0.1;
            Chose2 = false;
        } else {Chose2=true; }

    }

    public double addons(double ax)
    {
        if(Math.abs(ax) > controllerDeadzone)
        {
            ax = 0;
        }

        if(ax > Limit)ax = Limit;
        else if(ax < -Limit)ax = -Limit;

        return ax;

    }

    void MS(double x1, double x2, double x3, double x4){
        csint.BackLeft.setPower(x1);
        csint.FrontRight.setPower(x2);
        csint.FrontLeft.setPower(x3);
        csint.BackRight.setPower(x4);

    }


}
