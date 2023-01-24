package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoClaw {

    public Servo servo1;

    Color_Sensor cdetect = new Color_Sensor();
    public ElapsedTime timer = new ElapsedTime();
    public boolean open;


    Gamepad gamepad;


    public void init(HardwareMap ahwMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        servo1 = ahwMap.get(Servo.class, "Servo1");
        cdetect.init(ahwMap);
        Open();
        timer.reset();
        //servo2 = hwMap.get(Servo.class, "Servo2");
        //  Init();
        //Open();

    }

    public void Open(){
        servo1.setPosition(0.7); open = true;
    }

    public void Closed(){
        servo1.setPosition(0);
    }

    public void run()
    {
        cdetect.update();

        if((cdetect.whatColorIsIt() == 1 && timer.seconds() > 2) && open)
        {
            open = false;
            Closed();
        }

        if(gamepad.b)
        {
            timer.reset();
            Open();
        }
    }

    public int returncolor()
    {
        return cdetect.whatColorIsIt();
    }


}
