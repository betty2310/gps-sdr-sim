# GPT Image prompts

Mode: built-in GPT Image tool.

## F9P photo annotation

Edit target: user-supplied ZED-F9P-04B-01 photo.

Use case: precise-object-edit.
Asset type: an annotated reference photograph for a physical electronics wiring guide.
Input image 1 is the edit target, the user's actual ZED-F9P-04B-01 breakout board in their hand. Preserve the board, every connector, every pad, silkscreen text, component, and hand exactly as shown; do not redesign, reorient, or invent components or wires. Extend the canvas with a clean white margin if needed for readable callouts.
Add exactly four clean numbered callouts with thin leader lines and small circles precisely around these existing physical points:
1 orange: "PPS — signal to buffer input". Target the left-edge round solder pad labelled PPS, BETWEEN RTK above and RST below. In the original square photo it is approximately 20.7% from left, 59.8% from top. Absolutely do not point to RTK or RST.
2 dark gray: "GND — common ground". Target the topmost pad on the LEFT pad row labelled GND, immediately ABOVE 3V3, approximately 19.7% from left, 42.4% from top. Absolutely do not point to 3V3.
3 blue: "USB-C — board power and GNSS data". Target USB-C metal socket at top-left of board, approximately 31% from left, 30% from top.
4 teal: "SMA — GNSS antenna". Target large gold threaded antenna connector extending from BOTTOM of board, approx 43.8% from left, 81.9% from top, not the tiny u.FL jack.
Title: "F9P connection points"
Footer: "Solder only with USB power disconnected"
Keep printed PPS, GND, RTK, RST, 3V3 labels unobscured. This is a technically exact photo annotation, not a redesign. Do not draw a direct wire from PPS to USRP. No added pads. Use ample whitespace, legible English, professional lab manual style.

## USRP connector illustration

Generated schematic illustration; not a photograph or actual connector ordering.

Use case: scientific-educational.
Create a wide, exceptionally clear technical connector-identification illustration for a lab wiring guide. This is explicitly a SCHEMATIC port map, not a photograph or an accurate spatial reproduction of the X300 back panel.
Title: "USRP X300 — choose the labelled input"
Subtitle: "Connector positions below are schematic. Match the labels on your unit."
Show five separate, realistic gold SMA female panel sockets on a simple silver panel, each with its own clear printed label, with plenty of horizontal space:
"PPS/TRIG IN", "PPS/TRIG OUT", "REF IN", "REF OUT", "GPS".
Above PPS/TRIG IN add a green callout "CONNECT PPS HERE". Show a single black coax with gold SMA male threaded nut attached ONLY to PPS/TRIG IN; show a small orange signal highlight in the cable. Label cable "From the 5 V PPS buffer".
Above REF IN add a blue callout "10 MHz clock only" with a small blue arrow. Do not connect the PPS cable to REF IN.
Under PPS/TRIG OUT: "Output — do not connect F9P here".
Under REF OUT: "Clock output".
Under GPS: "Antenna input".
A smaller inset along the bottom, separated from panel: cutaway of a coax cable and SMA connector. Clearly label "Centre conductor = PPS" and "Outer shield = GND". Depict one insulated central conductor and surrounding braided shield, electrically separate, not shorted.
Footer: "0 to about 5 V logic pulse • 1 pulse per second • Common ground required"
Use professional white background, readable large text, restrained orange/blue/green highlights. Do not imply connector ordering is the real rear-panel ordering. No RF front-panel connectors, no invented USB sockets, no photographed test results.
