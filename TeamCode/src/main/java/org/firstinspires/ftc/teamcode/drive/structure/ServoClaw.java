package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoClaw {

    public Servo servo1;
    public Servo servo2;


    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        servo1 = hwMap.get(Servo.class, "Servo1");
        servo2 = hwMap.get(Servo.class, "Servo2");
    }

    public void Closed(){
        servo1.setPosition(-0.2);
        servo2.setPosition(0.13);
    }

    public void Open(){
        servo1.setPosition(0.13);
        servo2.setPosition(0);
    }
}
