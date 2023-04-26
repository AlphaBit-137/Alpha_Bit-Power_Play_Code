package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoClaw {

    public Servo servo1;
    public Servo centrationServo;
    public Servo rotationServo;

    Color_Sensor cdetect = new Color_Sensor();
    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime close_time = new ElapsedTime();
    public boolean open = true;

    public boolean toggle = true;

    Gamepad gamepad;


    public void init(HardwareMap ahwMap, Gamepad gamepad,boolean isAuto) {
        this.gamepad = gamepad;
        servo1 = ahwMap.get(Servo.class, "Servo1");
        // cdetect.init(ahwMap);
        
        Open();
        timer.reset();
        centrationServo = ahwMap.get(Servo.class, "CE_Servo");
        rotationServo = ahwMap.get(Servo.class, "RO_Servo");


         if(!isAuto) {
             startPos();
         }

         if(isAuto) {
             centrationServo.setPosition(0.6);
            //servo1.setPosition(0.2);
         }
    }

    public void pickedUpCone()
    {
        centrationServo.setPosition(0.36);
    }

    public void startPos()
    {
        rotationServo.setPosition(1);
        centrationServo.setPosition(0.25);
    }

    public void lowPose()
    {
        rotationServo.setPosition(1-0.02);
        centrationServo.setPosition(0.7);
    }


    public  void conePose()
    {
        rotationServo.setPosition(0);
        centrationServo.setPosition(0);
    }

    public void dropped()
    {
        rotationServo.setPosition(0.02);
        centrationServo.setPosition(8);
    }

    public void Open(){
        servo1.setPosition(0.5);
        open = true;
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
      /*  if(open) {

            cdetect.update();

            if (((cdetect.whatColorIsIt2() == 1 || cdetect.whatColorIsIt2() == 2) && timer.seconds() > 0.6 && cdetect.distance < 3) && open) {
                Closed();
            }

        }


        if(rotationServo.getPosition() < 0.02) {
            if (!open) {
                if (close_time.seconds() > 0.3) pickedUpCone();
            } else if (open) {
                startPos();
            }
        }*/

        if(gamepad.dpad_up || gamepad.dpad_right)
        {
            conePose();
        }

    if(gamepad.dpad_down)
    {
        startPos();
    }

        if(gamepad.dpad_left)
        {
            lowPose();
        }

     if(gamepad.b)
     {
         if(toggle)
         {
             if(open)
             {
                 Closed();
             }else Open();
         }
         toggle = false;
     }else toggle = true;


    }


    public void stackPose()
    {
        rotationServo.setPosition(0.63);
        centrationServo.setPosition(0.3);
    }

    public int returncolor()
    {
        return cdetect.whatColorIsIt2();
    }

    public double rg(){return cdetect.green;}

    public double rr(){return cdetect.red;}

    public double rb(){return cdetect.blue;}


}
