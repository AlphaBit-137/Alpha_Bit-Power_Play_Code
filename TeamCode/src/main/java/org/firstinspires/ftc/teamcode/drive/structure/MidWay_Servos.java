package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MidWay_Servos {

    Servo centrationServo;
    Servo rotationServo;

    Gamepad mid_gamepad;

    public void init(HardwareMap hwmpa, Gamepad mid_gamepad)
    {
        this.mid_gamepad = mid_gamepad;
        centrationServo = hwmpa.get(Servo.class, "CE_Servo");
        rotationServo = hwmpa.get(Servo.class, "RO_Servo");

        startPos();
    }

    public void startPos()
    {
        centrationServo.setPosition(0.5);
        rotationServo.setPosition(0.03);
    }

    public  void conePose()
    {
        centrationServo.setPosition(0.7);
        rotationServo.setPosition(0.75);
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
