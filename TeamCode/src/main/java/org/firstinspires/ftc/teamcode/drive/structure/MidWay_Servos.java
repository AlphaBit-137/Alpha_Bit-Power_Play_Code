package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MidWay_Servos {

    Servo upDownServo;
    Servo rotationServo;

    Gamepad mid_gamepad;

    public void init(HardwareMap hwmpa, Gamepad mid_gamepad)
    {
        upDownServo = hwmpa.get(Servo.class, "UPServo");
        rotationServo = hwmpa.get(Servo.class, "ROTServo");
    }

    public void startPos()
    {
        upDownServo.setPosition(0);
        rotationServo.setPosition(0);
    }

    public  void conePose()
    {
        upDownServo.setPosition(0.5);
        rotationServo.setPosition(0.5);
    }

    public void run()
    {
        if(mid_gamepad.dpad_up)
        {
            conePose();
        }else if(mid_gamepad.dpad_down)
        {
            startPos();
        }

    }


}
