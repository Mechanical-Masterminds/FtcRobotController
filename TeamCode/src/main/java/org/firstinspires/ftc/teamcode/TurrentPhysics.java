package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TurrentPhysics extends {
    public static double targetX = 0;
    public static double targetY = 0;
    public static double launchAngle = 0.1;
    public static double velocity = 0;
    public static double G = 32.174;

    public double computeRequiredVelocity() {
        launchAngle = -((37/118)*targetX)+80.0168;
        double cosA = Math.cos(launchAngle);
        double sinA = Math.sin(launchAngle);

        double inner = (-2 * targetY / G) + ((2 * targetX * sinA) / (G * cosA));

        // Safety check for negative sqrt
        if (inner <= 0) {
            throw new IllegalArgumentException(
                    "Invalid parameters: square root term is negative or zero (inner = " + inner + ")"
            );
        }
        double denominator = cosA * Math.sqrt(inner);
        velocity = targetX / denominator;
        telemetry.addData("XPos", targetX);
        telemetry.addData("YPos", targetY);
        telemetry.addData("Angle", launchAngle);
        telemetry.addData("Velocity", velocity);

