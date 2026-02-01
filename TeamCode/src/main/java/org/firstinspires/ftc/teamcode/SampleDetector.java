
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SampleDetector {
    public enum SAMPLE_COLOR {NAUGHT, PURPLE, GREEN }

    /** The colorSensor */
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;

    float gain = 2;

    /** HSV values from the sensor */
    final float[] hsvValues = new float[3];

    public SampleDetector(HardwareMap hardwareMap) {
        // get the color sensors
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");

        // turn on the light
        if (colorSensor1 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor1).enableLight(true);
            ((SwitchableLight)colorSensor2).enableLight(true);
        }

        // set the gain
        colorSensor1.setGain(gain);
        colorSensor2.setGain(gain);
    }

    public SAMPLE_COLOR getColor() {
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor1.getNormalizedColors();

        // could just base on colors.alpha() > 0.3;

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        // check the distance
        if (colorSensor1 instanceof DistanceSensor) {
            // distance in centimeters
            double dist = ((DistanceSensor)colorSensor1).getDistance(DistanceUnit.CM);

            // make sure the distance is less than some value
            if (dist > 6.0) {
                // sample too far away, so try other sensor
                colors = colorSensor2.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), hsvValues);
                dist = ((DistanceSensor)colorSensor2).getDistance(DistanceUnit.CM);

                if (dist > 6.0) {
                    return SAMPLE_COLOR.NAUGHT;
                }
            }
        }

        //  H S V are indices 0, 1, 2
        if (hsvValues[1] < 0.5) {
            // saturation is too low for a good reading
            return SAMPLE_COLOR.NAUGHT;
        }

        // now dispatch on hue
        if (hsvValues[0] < 210.0) {
            return SAMPLE_COLOR.GREEN;
        } else {
            return SAMPLE_COLOR.PURPLE;
        }

    }
}
