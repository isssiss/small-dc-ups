# Small DC UPS controller

UPS controller board

There are some old and various components because I designed this with the components I already had.

NOTE: Only one board was made and the schematic had an error. Fix can be seen in copper_side.png and it is fixed only in the schematic and pcb_v2_from_top.png.
Some components need to be installed on copper side.

In pcb_v2_from_top.png red lines are on pcb copper side, grey is ground (plane on copper side after milling the traces) and blue is  jumpers (board component side).


* ups_1_4_source.pdf

  Power connections and output current measurement.
  Use low dropout diode for D1 (MBR1535CT).

* ups_2_4_charge.pdf

  Battery charger and charge current measurement.
  Charge current can be adjusted with R17/R18.
  Adjust R23 to correct charge end voltage without a connected battery. Measure at D8 cathode.
  See components.png for R14-R15 placement.

* ups_3_4_control.pdf

  Arduino and fan control.

* ups_4_4_testload.pdf

  Battery test load.
  See components.png for R47-R49 placement.
