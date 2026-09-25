# Additional physical wiring illustrations

Mode: built-in GPT Image tool.

These are proposed assembly illustrations, not photographs of a tested build. The generic buffer enclosure is conceptual. A buffer is an interface design option; direct connection requires verified voltage, loading and waveform. The final images were visually reviewed for signal and ground routing, solder-pad identity, connector contacts and waveform labels.

## F9P physical assembly

Use case: precise-object-edit.
Create a technically clear, photorealistic educational mockup of a proposed physical F9P PPS assembly, with visible wire shapes, solder joints and SMA output. Input image is the user's actual F9P board, the edit target. Preserve its component layout, labels, orientation and connectors; replace hand/background with white workbench.
THIS REVISION MUST MAKE THE WIRING UNAMBIGUOUS. Composition: F9P BOARD ON RIGHT HALF, GENERIC PPS BUFFER BOX ON LEFT HALF. The left-edge pads of the board face the right side of the buffer, so the wires have a very short, unobstructed path across the white gap. Board upright, USB-C top, antenna SMA bottom. No wire may pass behind or across the board. No wire may leave the canvas.
Title: "One proposed build: F9P + PPS buffer"
Subtitle: "Physical mockup — verify electrical compatibility before use"
Draw exactly two continuous insulated flexible wires between the board and the buffer:
1 BLACK from the actual LEFT GND pad immediately ABOVE 3V3 (photo approx x19.7%,y42.4%) across the gap to a single buffer input terminal on its RIGHT edge labelled "GND". Put this terminal ABOVE the IN terminal.
2 ORANGE from the actual LEFT PPS pad BETWEEN RTK and RST (photo approx x20.7%,y59.8%) across the gap to the LOWER buffer input terminal labelled "IN".
Each wire must be visibly soldered ONLY to its correct board pad with a tiny exposed tinned tip and small solder fillet. At the other end it must visibly enter its own matching terminal opening. Draw the wires with gentle curved slack and route together where possible. The two wires never touch each other's exposed metal. NO wires on any RIGHT board pads. Absolutely no connection to RTK, RST, 3V3, CS, TX or RX.
The generic opaque buffer box on left is conceptual, not a real photographed commercial product. Label its face "PPS buffer" and "5 V supply". Its black GND input is on the right-upper edge, its orange IN input on right-lower edge. On TOP of buffer show a separate 2-pole power terminal: a red wire visibly attached to "+" and a black power wire visibly attached to "−", both exiting upwards; callout "Separate regulated 5 V".
The buffer output is on its LEFT edge. Show a single black coax cable curving from this output to a newly added SMA female bulkhead jack mounted in a small separate plastic bracket at the FAR LEFT. Label "Added PPS OUT". SMA female must have external threads, retaining hex nut and a central socket/hole. Coax must connect behind this new jack. This new output is clearly separate from the F9P antenna connector.
Show a small off-board clip holding the two input wires' insulation for strain relief, away from solder joints.
Callouts with precise leaders: "BLACK: GND"; "ORANGE: PPS"; "Small solder joints"; "Cable strain relief"; "Coax output"; "Existing antenna input" pointing ONLY to the bottom F9P connector; "USB power + data" pointing to USB-C.
Along lower margin show one large realistic standard SMA male cable plug, with threaded coupling nut and centre PIN, beside the new female jack detail with centre SOCKET. Label "SMA male cable → X300 PPS/TRIG IN".
Footer: "The buffer is a design option; direct wiring requires verified voltage, load and waveform."
White backdrop, large legible simple English, generous spacing, physical wires with real thickness and bends, no hidden wire routing, no dense schematic, no invented connections. Keep the board substantially faithful to reference.

## Wire, SMA connector and pulse close-ups

undefined

