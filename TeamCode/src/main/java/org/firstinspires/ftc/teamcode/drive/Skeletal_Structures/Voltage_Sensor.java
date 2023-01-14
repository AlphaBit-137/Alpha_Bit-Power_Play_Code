package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Voltage_Sensor {

    VoltageSensor Vsensor;

    public void init(HardwareMap ahwmap)
    {
        Vsensor = ahwmap.voltageSensor.iterator().next();
    }

    public double GetCompensation()
    {
        if(Vsensor.getVoltage() > 12)
        {
            return 12/Vsensor.getVoltage();
        }else return 1;
    }

}
