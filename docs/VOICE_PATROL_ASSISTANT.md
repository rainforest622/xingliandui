# Voice Patrol Assistant

The ASRPRO board is the robot's offline command and local speech-output
device. It does not send motor frames. Every recognised command follows this
path:

```text
ASRPRO command byte -> Raspberry Pi voice_service.py -> rover_arbiter.py
-> existing WS63 / WAVE ROVER safety path
```

The Raspberry Pi is the only writer to the ASRPRO UART. This prevents a speech
reply from competing with an incoming recognition command.

## Implemented Functions

| Function | Voice input | ASRPRO byte | Pi action | Local reply |
| --- | --- | --- | --- | --- |
| Start patrol | Start patrol | `A1` | Starts the imported map route | `F1` |
| Pause / stop | Pause patrol / Stop patrol | `A2` / `A4` | Stops route, returns to manual standby | `F2` / `F3` |
| Environment query | Report temperature and humidity | `A8` | Returns the latest WS63 temperature/humidity snapshot | `E0 01 ... CRC` |
| Distance query | Front distance | `A9` | Returns the latest ultrasonic snapshot | `E0 02 ... CRC` |
| Patrol query | Patrol progress | `AA` | Returns current map step and loop in `/status` | `F4`, `FC` |
| Battery query | Report battery | `AB` | Explicitly reports that no BMS is installed | `F4`, `FD` |
| Obstacle alert | Automatic | n/a | De-duplicated while automatic patrol is active | `F7` |
| Temperature alert | Automatic | n/a | Triggered at >= 35.0 C | `F8` |
| Humidity alert | Automatic | n/a | Triggered at >= 85.0 %RH | `F9` |
| Patrol complete | Automatic | n/a | Reports normal or environmental-alert completion | `FA` / `FB` |

`F7` through `FE` are fixed proactive/fallback reply codes from the Pi. `E0`
starts a seven-byte dynamic telemetry frame: `E0, kind, valueA_hi, valueA_lo,
valueB_hi, valueB_lo, xor`. The environment frame preserves WS63's deci-units,
so `238,652` is spoken as "current temperature 23.8 degrees Celsius; current
humidity 65.2 percent". The ASRPRO combines locally generated digit and unit
clips, therefore this remains offline after the speech model is compiled.

The WS63 publishes a
standard `+ROBOT:MON` telemetry frame every five seconds and immediately when
the alarm bitmask changes. The Pi suppresses repeated obstacle announcements
for four seconds and environmental announcements for twelve seconds.

## ASRPRO Model Update

1. Open `raspberry_pi/asrpro/robot_voice_control_baidu.hd` in Tianwen Block
   professional programming mode.
2. Open `robot_voice_control_pro.ino` in the code panel and retain the prompt
   directives `11026` through `11047`. They are the sensor-unavailable,
   temperature/humidity/distance, digit, and number-unit clips required for
   the live-value response.
3. Generate the voice model, compile/download with the ASRPRO 2M target, then
   press RESET once.
4. Connect the module to the Pi as `/dev/nearlink-asr` at 9600 bps and restart
   `nearlink-voice.service`.

The source mirror `robot_voice_control_pro.ino` has the same byte protocol and
prompt text. The `.hd` project is the normal artifact to compile because it
also generates the local TTS prompt assets.

## Verification

On the Pi, inspect the two independent loops:

```bash
sudo systemctl restart nearlink-rover-arbiter nearlink-voice
sudo journalctl -fu nearlink-voice
curl http://127.0.0.1:8090/voice/latest
```

Test in this order: report temperature and humidity, confirm the module says
the numbers displayed by `curl http://127.0.0.1:8090/voice/latest`, place an
obstacle during an automatic route, then complete one route loop. The Harmony
app's inspection panel now polls `/voice/latest` even when automatic inspection
is off, so a voice query updates its distance, temperature, humidity, and alarm
widgets from the same Pi/WS63 snapshot.

Battery percentage is intentionally unavailable until a real BMS or ADC
current/voltage measurement is connected. The assistant never fabricates a
percentage from motor commands.
