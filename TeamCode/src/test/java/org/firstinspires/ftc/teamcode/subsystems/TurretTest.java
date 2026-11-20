package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.RobotConfig.TurretConfig;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class TurretTest {
    private static final double MAX_V = 3.3, ZERO_V = 1.65, VOLTS_PER_DEG = MAX_V / 360;
    private double zero, ratio, kP, kI, kD, kS;
    private AnalogInput enc;
    private CRServo servo;
    private Turret turret;

    @Before
    public void setUp() {
        zero = TurretConfig.ENCODER_ZERO_V;
        ratio = TurretConfig.ENCODER_RATIO;
        kP = TurretConfig.kP; kI = TurretConfig.kI; kD = TurretConfig.kD; kS = TurretConfig.kS;
        TurretConfig.ENCODER_ZERO_V = ZERO_V;
        TurretConfig.ENCODER_RATIO = 1;
        TurretConfig.kP = 0.02; TurretConfig.kI = 0; TurretConfig.kD = 0; TurretConfig.kS = 0;

        enc = mock(AnalogInput.class);
        when(enc.getMaxVoltage()).thenReturn(MAX_V);
        servo = mock(CRServo.class);
        turret = new Turret(servo, enc);
    }

    @After
    public void restore() {
        TurretConfig.ENCODER_ZERO_V = zero;
        TurretConfig.ENCODER_RATIO = ratio;
        TurretConfig.kP = kP; TurretConfig.kI = kI; TurretConfig.kD = kD; TurretConfig.kS = kS;
    }

    private void read(double turretDeg) {
        when(enc.getVoltage()).thenReturn(ZERO_V + turretDeg * VOLTS_PER_DEG);
        turret.periodic();
    }

    @Test
    public void readsTurretAngle() {
        read(90);
        assertEquals(90, Math.toDegrees(turret.measuredAngle()), 1e-6);
    }

    @Test
    public void countsShaftWrapsWhenGeared() {
        TurretConfig.ENCODER_RATIO = 0.25;
        // shaft walks 0 -> 170 -> -170 (= 190) -> -10 (= 350): turret 87.5
        for (double shaft : new double[]{0, 170, -170, -10}) {
            when(enc.getVoltage()).thenReturn(ZERO_V + shaft * VOLTS_PER_DEG);
            turret.periodic();
        }
        assertEquals(87.5, Math.toDegrees(turret.measuredAngle()), 1e-6);
    }

    @Test
    public void negativeRatioFlipsSign() {
        TurretConfig.ENCODER_RATIO = -1;
        read(30);
        assertEquals(-30, Math.toDegrees(turret.measuredAngle()), 1e-6);
    }

    @Test
    public void drivesTowardTheCommand() {
        turret.setAngle(Math.toRadians(45));
        read(0);
        assertEquals(0.9, turret.power(), 1e-9);
        assertFalse(turret.settled());
        read(45);
        assertEquals(0, turret.power(), 1e-9);
        assertTrue(turret.settled());
    }

    @Test
    public void softLimitBlocksPushingFurtherOut() {
        TurretConfig.kP = 0;
        TurretConfig.kS = 0.5; // outward push only from kS
        turret.setAngle(Math.toRadians(TurretConfig.MAX_DEG));
        read(TurretConfig.MAX_DEG - 10);
        assertTrue(turret.power() > 0);
        read(TurretConfig.MAX_DEG + 1);
        assertTrue(turret.power() <= 0);
    }

    @Test
    public void noEncoderStaysUnpowered() {
        Turret t = new Turret(servo, null);
        t.setAngle(1);
        t.periodic();
        assertEquals(0, t.power(), 0);
        assertTrue(t.settled());
        verify(servo).setPower(0);
        verify(servo, never()).setPower(1);
    }

    @Test
    public void openLoopPassesPowerButKeepsLimits() {
        turret.setOpenLoop(0.3);
        read(0);
        assertEquals(0.3, turret.power(), 1e-9);
        read(TurretConfig.MAX_DEG + 1);
        assertEquals(0, turret.power(), 1e-9);
        turret.setOpenLoop(Double.NaN);
        read(0);
        assertEquals(0, turret.power(), 1e-9);
    }
}
