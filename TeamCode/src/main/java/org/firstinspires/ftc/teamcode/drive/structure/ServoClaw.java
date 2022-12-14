package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoClaw {

    public Servo servo1;
    public Servo servo2;

    Gamepad gamepad;
    int poz = 1;
    public boolean turns = false;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        hwMap = ahwMap;
        servo1 = hwMap.get(Servo.class, "Servo1");
        servo2 = hwMap.get(Servo.class, "Servo2");
        Init();
        Open();
    }

    public void Init()
    {
        servo1.setPosition(0.1);
        servo2.setPosition(0.1);
    }

    public void Closed(){
        servo1.setPosition(-0.2);
        servo2.setPosition(0.13);
    }

    public void Open(){
        servo1.setPosition(0.13);
        servo2.setPosition(0);
    }

    public void run()
    {
        if(gamepad.b){
            if(poz==1 && turns==true){
                Open();
                poz=2;
            }else if(poz==2 && turns==true){
                Closed();
                poz=1;
            }
            turns=false;
        }else turns=true;
    }


}
