# Original source code (kept separate from simulations)

- `Arduino/SEVEN_color_sorter_original/SEVEN_color_sorter_original.ino`: byte-for-byte copy of the user's original Arduino sketch from the attached `Pasted text.txt`. An unchanged `.txt` copy is also provided under `Arduino/Arduino_original_paste.txt`.
- `RaspberryPi/color_sorter_original.py`: faithful transcription of the full Raspberry Pi Python code the user pasted in the conversation. No standalone `.py` upload was supplied, so original-file byte identity cannot be proven; the source logic, comments, threshold constants, and command strings were not intentionally changed.
- `SOURCE_MANIFEST.json`: SHA-256 fingerprints for both included files.

**Do not replace these files with upstream GitHub firmware.** The upstream sketch uses different commands. The real camera script sends `CHECK RED`, `ARM`, and `RUN RED` (similarly for green/blue) to the team's own Arduino sketch.

Hardware warning: original sketch uses general 0–180° commanded limits, not measured mechanical limits. The serial `STOP` command holds servo output; it does not physically disconnect servo power. Recalibrate and ensure clearance before activating motors.


**Bin capacity tracking.** The optional `RaspberryPi/sort_with_slots.py` controller uses the original HSV rules and firmware destinations. It adds persistent counters, confirmed reset and recovery around `CHECK`, `ARM` and `RUN`. [Controller and FSM specification](../docs/SLOT_TRACKING_AND_FSM.md)
