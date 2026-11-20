package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;

@Autonomous(name = "Blue Far - 2 Tip", group = "Autonomous")
public class BlueFar2Tip extends Far2Tip {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
