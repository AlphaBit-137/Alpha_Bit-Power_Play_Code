package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

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

    public void run()
    {

        double x = Range.clip(-gamepad.left_stick_x * 1.1,-Limit,Limit);
        double y = Range.clip(gamepad.left_stick_y,-Limit,Limit);
        double rx = Range.clip(-gamepad.right_stick_x,-Limit,Limit);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double Drive3 = (y + x + rx) / denominator;
        double Drive1 = (y - x + rx) / denominator;
        double Drive2 = (y - x - rx) / denominator;
        double Drive4 = (y + x - rx) / denominator;

        MS(Drive1, Drive2, Drive3, Drive4);

        if(gamepad.x) {
            if(Chose)Limit=Limit+0.1;
            if(Limit>1)Limit=1;
            Chose = false;
        } else Chose=true;

        if(gamepad.y) {
            if(Chose2)Limit=Limit-0.1;
            if(Limit<0.1)Limit=0.1;
            Chose2 = false;
        } else {Chose2=true; }

    }

    void MS(double x1, double x2, double x3, double x4){
        csint.BackLeft.setPower(x1);
        csint.FrontRight.setPower(x2);
        csint.FrontLeft.setPower(x3);
        csint.BackRight.setPower(x4);
    }





}
