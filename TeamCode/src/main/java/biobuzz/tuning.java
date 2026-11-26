package biobuzz;

import com.acmerobotics.dashboard.config.Config;


@Config
public class tuning {
    public static double shootkp = 0.015;
    public static double shootki = 0.0;
    public static double shootkd = 0.0;
    public static double shootkf = 0.00005;
    public static double shootmax = 1.0;
    public static double shootmin = 0.0;
    public static double targetvelo = 700;
    //values at 10.60 volts of batt power (during transfer)

    public static double hoodkp = -0.01; //0.001
    public static double hoodki = -0.01;
    public static double hoodkd = 0;

    public static double startrotoffset = 0;
}
