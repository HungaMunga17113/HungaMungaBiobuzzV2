package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;

@Autonomous(name = "Red Far - 2 Tip", group = "Autonomous")
public class RedFar2Tip extends Far2Tip {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
