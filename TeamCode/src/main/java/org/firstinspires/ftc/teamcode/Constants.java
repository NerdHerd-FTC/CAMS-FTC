package org.firstinspires.ftc.teamcode;

public final class Constants {
    public static final double COUNTS_PER_MOTOR_REV = 28;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    public static final double DRIVE_GEAR_REDUCTION = 20;   //gear ratio
    public static final double WHEEL_DIAMETER_INCH = 3.65;    // For figuring circumference: 90mm
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * Math.PI);

    public static final double ARM_POWER = 0.65; //for quick adjustments

    public static final int  HIGH_JUNCTION_TICKS = 690;
    public static final int  MEDIUM_JUNCTION_TICKS = 420;
    public static final int  LOW_JUNCTION_TICKS = 290;
}
