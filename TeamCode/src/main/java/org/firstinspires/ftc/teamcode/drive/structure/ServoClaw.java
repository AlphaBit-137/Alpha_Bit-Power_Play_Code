package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoClaw {

    public Servo servo1;
    Servo centrationServo;
    Servo rotationServo;

    Color_Sensor cdetect = new Color_Sensor();
    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime close_time = new ElapsedTime();
    public boolean open = true;

    Gamepad gamepad;


    public void init(HardwareMap ahwMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        servo1 = ahwMap.get(Servo.class, "Servo1");
        cdetect.init(ahwMap);
        Open();
        timer.reset();
        centrationServo = ahwMap.get(Servo.class, "CE_Servo");
        rotationServo = ahwMap.get(Servo.class, "RO_Servo");

        startPos();

    }

    public void pickedUpCone()
    {
        centrationServo.setPosition(0.27);
    }

    public void startPos()
    {
        rotationServo.setPosition(0.01);
        centrationServo.setPosition(0.38);
    }

    public  void conePose()
    {
        rotationServo.setPosition(0.69);
        centrationServo.setPosition(0.7);
    }

    public void Open(){
        servo1.setPosition(0.5); open = true;
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


        if(rotationServo.getPosition() < 0.1) {
            if (!open) {
                if (close_time.seconds() > 0.3) pickedUpCone();
            } else if (open) {
                startPos();
            }
        }

        if(gamepad.dpad_up)
        {
            conePose();
        }

        if(gamepad.dpad_down)
        {
            startPos();
        }

        if(gamepad.b)
        {
            timer.reset();
            Open();
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
