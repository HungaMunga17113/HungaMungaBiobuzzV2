[pedro](http://192.168.43.1:10158), [dash](http://192.168.43.1:8080/dash)

```sh
adb connect 192.168.43.1:5555
```

### sw todo list once we get robot:
1) tune pedro
2) configure all servos
3) run flywheel svatuner + tune pid
4) tune turret (ENCODER_ZERO_V, ENCODER_RATIO on open loop; then kS + PID)
5) shot table (for shooting wherever we wanna)