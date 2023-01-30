package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Voltage_Sensor {

    VoltageSensor Vsensor;

    public double normalizer;

    public void init(HardwareMap ahwmap , double normalizer)
    {
        Vsensor = ahwmap.voltageSensor.iterator().next();
        this.normalizer = normalizer;
    }

    public double GetCompensation()
    {
        if(Vsensor.getVoltage() > 12)
        {
            return 12/Vsensor.getVoltage();
        }else return 1;
    }

}
