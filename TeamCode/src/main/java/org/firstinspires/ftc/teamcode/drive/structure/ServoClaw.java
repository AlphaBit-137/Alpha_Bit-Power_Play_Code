package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoClaw {

    public Servo servo1;
    public Servo servo2;


    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        servo1 = hwMap.get(Servo.class, "Servo1");
        servo1.setPosition(0);
        servo2 = hwMap.get(Servo.class, "Servo2");
        servo2.setPosition(0);
    }

    public void Closed(){
        servo1.setPosition(0.33);
       // servo2.setPosition(0.33);
    }

    public void Open(){
        servo1.setPosition(0);
       // servo2.setPosition(0);
    }
}
