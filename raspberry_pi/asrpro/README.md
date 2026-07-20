# ASRPRO Live Telemetry Voice Build

`robot_voice_control_pro.ino` is the source of truth for the ASRPRO
professional-mode build. It adds the local prompts and frame decoder needed to
speak measured values from the Raspberry Pi instead of a fixed sentence.

## One-Time Reflash

1. Stop the Raspberry Pi services or unplug the ASRPRO Type-C data cable from
   the Pi, then connect the ASRPRO board directly to the PC with a data cable.
2. Start `D:\天问Block\TwenBlock.exe`, select ASRPRO professional programming
   mode, and open/paste `robot_voice_control_pro.ino` in the code panel.
3. Generate the Baidu TTS assets from every `playid:11026` through
   `playid:11047` directive. These are the offline sensor labels, digits, and
   number units used at runtime.
4. Select the ASRPRO 2M target and the newly appearing COM port, then compile
   and download. Press RESET once after the download completes.
5. Reconnect the ASRPRO board to the Pi and run:

   ```bash
   sudo systemctl restart nearlink-rover-arbiter nearlink-voice
   ```

## Expected Result

With a WS63 value of `temperature_deci_c=260` and
`humidity_deci_percent=600`, saying `报告温湿度` produces a local reply
equivalent to: `当前温度二十六点零摄氏度，当前湿度六十点零百分比。`

The Pi sends a checksummed `E0` frame; it never sends motor commands as part
of a telemetry query.
