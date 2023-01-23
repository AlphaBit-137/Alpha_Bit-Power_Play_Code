package org.firstinspires.ftc.teamcode.drive.structure;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Color_Sensor {

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    float saturation;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;

    int relativeLayoutId;
    View relativeLayout;

    public void init(HardwareMap hwmap) {
        sensorColor = hwmap.get(ColorSensor.class, "sensor_color_distance");

        sensorDistance = hwmap.get(DistanceSensor.class, "sensor_color_distance");


        relativeLayoutId = hwmap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hwmap.appContext).findViewById(relativeLayoutId);

    }


        public void updateSensor() {

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);


            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            saturation = hsvValues[1];

        }


    public int whatColorIsIt() {
        if (sensorColor.green() > (sensorColor.blue() + sensorColor.red()) / 1.42) {
            return 3; //Neither blue nor red
        } else if (sensorColor.red() < sensorColor.blue() && saturation >= 0.30) {
            return 2; //blue
        } else {
            return 1; //red
        }
    }

}
