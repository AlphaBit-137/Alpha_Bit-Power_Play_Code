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

        double Front, Turn, Sum, Diff, Side, Drive1, Drive2, Drive3, Drive4;

        Turn = Range.clip(gamepad.left_stick_x, -Limit, Limit);
        Front = Range.clip(gamepad.right_stick_y, -Limit, Limit);
        Side = Range.clip(gamepad.right_stick_x, -Limit, Limit);


        Sum = Range.clip(Front + Side, -1.0, 1.0);
        Diff = Range.clip(Front - Side, -1.0, 1.0);

        Drive1 = Range.clip(Sum - 2*Turn, -1.0, 1.0);
        Drive2 = Range.clip(Sum + 2*Turn, -1.0, 1.0);
        Drive3 = Range.clip(Diff - 2*Turn, -1.0, 1.0);
        Drive4 = Range.clip(Diff + 2*Turn, -1.0, 1.0);

        MS(Drive1, Drive2, Drive3, Drive4);

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

    void MS(double x1, double x2, double x3, double x4){
        csint.BackLeft.setPower(x1);
        csint.FrontRight.setPower(x2);
        csint.FrontLeft.setPower(x3);
        csint.BackRight.setPower(x4);

    }


}
