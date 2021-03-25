package org.firstinspires.ftc.teamcode;

public class Constants {
    public final int shooterPower = 1500;
    public final double clampDown = 0.6;
    public final double clampUp = 0;
    public final double armDown = 0.9;
    public final double armUp = 0.55;
    public final String vuforiaKey = "AR5tugH/////AAAAmZWCOAi6Vk4zplw3uCD3PmgKbNphk1fcCZOjab9YzLoNQfafXMjye9dox6cEGiBcmfnWt7eZ8ZOdNUKn10YmfgnbM9ntclwrxuEgimv7R3zusivZOQytR+bzWylYXXovLOi1dsgFu6Rm+rJ95sKy6yn/KkrD5nhhCShwxniD5t5FYkxGC7TY681Tx4jfCdnq4aU7tuzOiPnaG8uhaRElSGelPtIBjmBkSHKE8LxZuza3Aewwm2I/v5p5vPrCmDRJlzkWdCWnFZ4v/jmtZAnpR3lx+zA51ZAd9EGanlLSzKQa5C9BtOr9mfLFBKZNxmORyeWX7btz8prPOCeYQyhwtfn6QjVygV2IrCNh7iyL7838";

    //encoder stuff
    //points per revolution
    private final double PPR = 384.5;
    //mecanum diameter in mm
    private final double diameter = 96;
    //millimeters per inch
    private final double mmPI = 25.4;
//    public final double ticksPerTok = PPR / (diameter / mmPI * Math.PI);
    public final double ticksPerTok = 3780 / (24 * 5);
}
