# Small DC UPS for Home Assistant/Frigatenvr project

This is designed to power Asrock J1900M-ITX board with a PicoPSU automotive power supply with an input voltage 8...36V.
Mains power is provided by a laptop power supply with 19.5V output.

Backup battery is a 12V 9Ah lead acid gel battery (Ultracell UL-9-12).

Charger is a traditional standby current and voltage limited linear model.

Can be used with standard Network UPS Tools (NUT) with a blazer_ser driver and Megatec ascii protocol.

NOTE: I made only one pcb and the first version had an error. The error is corrected only in the schematic and v2 pcb image.

### Warning:

The output switch IRF9530 burned after a few months in use. Both IRF9530 were changed to IRF4905 which has much lower internal resistance and higher current rating. Zener diodes were also added in parallel of every fets 47k pullup resistor to limit the gate voltage to about 13V (2x 6.2V zeners in series).

Arduino LM340 voltage regulator is changed to a TSR 1-2540 dc converter because the original will run quite warm with 20V input.


Probably still contains errors and bugs.

I'm not responsible of any possible damage caused by any content stored here.

Use at your own risk, you have been warned.
