A real receiver starts with the waveform and asks:

Which PRNs are present?
What are their code delays and Dopplers?
What nav data do they carry?
What receiver position/time best explains those measurements?

gps-sdr-sim does the opposite. In gpssim.c, it starts with the receiver state already known:

receiver position xyz
scenario GPS time g0
satellite ephemeris from RINEX

Then it asks:

If the receiver were really here, at this time, what would each satellite signal look like at the antenna?
What code delay, Doppler, nav bit, and amplitude would each PRN have?
What I/Q samples would produce exactly that observation?

The Mental Model

The best way to think about it is:

known receiver state + known satellite orbits
    -> predicted measurements
    -> predicted waveform

A receiver does:

measured waveform
    -> measured pseudoranges/Dopplers
    -> estimated receiver state

So the simulator is a forward physical model of GPS signal generation.

The Flow In gpssim.c

1. Fix the receiver state

In main(), the program first fixes:

receiver position from -c, -l, -u, -x, or -g
scenario start time from -t, -T, or ephemeris default
ephemeris data from readRinexNavAll()

At this point, the simulator knows “where the receiver is” and “where satellites should be.”

2. Propagate each satellite orbit to that time

For each PRN, satpos() uses broadcast ephemeris to compute:

satellite ECEF position
satellite velocity
satellite clock correction

This is the same orbital information a receiver would eventually decode from the nav message, but here the simulator uses it directly.

3. Compute what the receiver would measure

Then computeRange() computes the signal path from that satellite to the known receiver:

geometric range
light-time delay
Earth rotation correction
ionospheric delay
satellite clock correction
range rate
azimuth/elevation

This produces the key observables:

pseudorange rho.range
pseudorange rate rho.rate

These are basically the “answers” a receiver tries to infer from the signal.

4. Convert those observables into signal timing

Next, computeCodePhase() turns geometry into signal parameters:

carrier Doppler:
chan->f_carr = -rhorate / LAMBDA_L1;
code rate:
chan->f_code = CODE_FREQ + chan->f_carr * CARR_TO_CODE;
current C/A code phase
current nav-bit index and word index

This is the key inversion step in your question:

receiver view: measure Doppler and code delay from signal
simulator view: compute Doppler and code delay from known geometry

5. Generate the message and PRN code

Each channel also gets:

a PRN-specific C/A code from codegen()
GPS nav subframes from eph2sbf()
parity/TOW-updated nav words from generateNavMsg()

So for each satellite, the simulator knows at that instant:

which nav bit should be on air
which PRN chip should be on air
what carrier phase/frequency should be on air

6. Turn that into the actual waveform

In the inner sample loop in main(), each sample is built as:

ip = chan[i].dataBit * chan[i].codeCA * cosTable512[iTable] * gain[i];
qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * gain[i];

That means each satellite contributes:

nav data sign
times C/A code sign
times carrier cosine/sine
times gain

Then all satellites are summed into one composite baseband signal:

i_acc += ip;
q_acc += qp;

That sum is what the receiver would have seen at its antenna if it were truly at the input location and time.

What “At This Time” Really Means

There are two time scales in the simulator.

Slow time: every 0.1 s

The outer loop updates geometry every 0.1 seconds in main():

recompute range
recompute Doppler
recompute code phase
recompute gain

So every 0.1 s, the simulator refreshes the physical state of the sky.

Fast time: every sample

Inside that 0.1 s block, the inner loop generates samples at the SDR sample rate, typically 2.6 MHz, in main().

For each sample it advances:

code phase
carrier phase
nav-bit timing

So the signal evolves continuously between the 0.1 s geometry updates.

Why This Produces A Valid GPS-Like Signal

It works because everything is self-consistent.

The same ephemeris drives:

satpos()
computeRange()
computeCodePhase()
eph2sbf()
generateNavMsg()

So the receiver sees a waveform whose:

code delay matches the geometry
Doppler matches the geometry
nav bits describe the same orbit used for the geometry

That is why a receiver can lock onto the synthetic signal and solve back toward the input position.
