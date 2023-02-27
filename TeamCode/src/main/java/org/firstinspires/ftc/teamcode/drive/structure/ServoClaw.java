package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoClaw {

    public Servo servo1;

    Color_Sensor cdetect = new Color_Sensor();
    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime close_time = new ElapsedTime();
    public boolean open = true;

    public boolean toggle = true;

    Gamepad gamepad;


    public void init(HardwareMap ahwMap, Gamepad gamepad,boolean isAuto) {

        this.gamepad = gamepad;
        servo1 = ahwMap.get(Servo.class, "Servo1");
        cdetect.init(ahwMap);
        
        Open();

        timer.reset();

    }

    public void Open(){
        servo1.setPosition(0.5); open = true;
    }

    public void dClaw()
    {
        servo1.setPosition(0.7); open = true;
    }

    public void Closed(){
        servo1.setPosition(0);
        open = false;
        close_time.reset();
    }

    public void run()
    {
        if(open) {

            cdetect.update();

            if (((cdetect.whatColorIsIt2() == 1 || cdetect.whatColorIsIt2() == 2) && timer.seconds() > 0.6 && cdetect.distance < 3) && open) {
                Closed();
            }

        }

        if(gamepad.b)
        {
            timer.reset();
            Open();
        }

        if(gamepad.x)
        {
            //dClaw();
            //dropped();
        }

        if(gamepad.a)
        {
            Closed();
        }
    }


    public int returncolor()
    {
        return cdetect.whatColorIsIt2();
    }

    public double rg(){return cdetect.green;}

    public double rr(){return cdetect.red;}

    public double rb(){return cdetect.blue;}


}
