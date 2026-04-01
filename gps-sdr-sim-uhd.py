#!/usr/bin/env python3
# a small script to transmit simulated GPS samples via UHD
# (C) 2015 by Harald Welte <laforge@gnumonks.org>
# Licensed under the MIT License (see LICENSE)

from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import time

class top_block(gr.top_block):

    def __init__(self, options):
        gr.top_block.__init__(self, "GPS-SDR-SIM")

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_sink = uhd.usrp_sink(options.args, uhd.stream_args(cpu_format="fc32"))
        self.uhd_usrp_sink.set_samp_rate(options.sample_rate)
        self.uhd_usrp_sink.set_center_freq(options.frequency, 0)
        self.uhd_usrp_sink.set_gain(options.gain, 0)
        self.uhd_usrp_sink.set_clock_source(options.clock_source)

        if options.bits == 16:
            # Supports regular files and named FIFOs for live streaming.
            self.blocks_file_source = blocks.file_source(
                gr.sizeof_short * 1, options.filename, options.repeat
            )

            # convert from interleaved short to complex values
            self.blocks_interleaved_short_to_complex = blocks.interleaved_short_to_complex(False, False)

            # establish the connections
            self.connect((self.blocks_file_source, 0), (self.blocks_interleaved_short_to_complex, 0))

        else:
            self.blocks_file_source = blocks.file_source(
                gr.sizeof_char * 1, options.filename, options.repeat
            )

            # convert from signed bytes to short
            self.blocks_char_to_short = blocks.char_to_short(1)

            # convert from interleaved short to complex values
            self.blocks_interleaved_short_to_complex = blocks.interleaved_short_to_complex(False, False)

            # establish the connections
            self.connect((self.blocks_file_source, 0), (self.blocks_char_to_short, 0))
            self.connect((self.blocks_char_to_short, 0), (self.blocks_interleaved_short_to_complex, 0))

        # scale complex values to within +/- 1.0 so don't overflow USRP DAC
        # scale of 1.0/2**11 will scale it to +/- 0.5
        self.blocks_multiply_const = blocks.multiply_const_vcc((1.0/2**11, ))

        # establish the connections
        self.connect((self.blocks_interleaved_short_to_complex, 0), (self.blocks_multiply_const, 0))
        self.connect((self.blocks_multiply_const, 0), (self.uhd_usrp_sink, 0))

def get_options():
    parser = OptionParser(option_class=eng_option)
    parser.add_option("-x", "--gain", type="eng_float", default=0,
                      help="set transmitter gain [default=0]")
    parser.add_option("-f", "--frequency", type="eng_float", default=1575420000,
                      help="set transmit frequency [default=1575420000]")
    # On USRP2, the sample rate should lead to an even decimator
    # based on the 100 MHz clock.  At 2.5 MHz, we end up with 40
    parser.add_option("-s", "--sample-rate", type="eng_float", default=2600000,
                      help="set sample rate [default=2600000]")
    parser.add_option("-t", "--filename", type="string", default="gpssim.bin",
                      help="set input file/FIFO name [default=gpssim.bin]")
    parser.add_option("-b", "--bits", type="eng_float", default=16,
                      help="set size of every sample [default=16]")
    parser.add_option("-a", "--args", type="string", default="",
                      help="set UHD arguments [default='']")
    parser.add_option("-c", "--clock_source", type="string", default="internal",
                      help="set clock source [default='internal']")
    parser.add_option("-r", "--repeat", action="store_true", default=False,
                      help="repeat the input source (disabled by default)")
    parser.add_option("--interactive", action="store_true", default=False,
                      help="wait for Enter to stop instead of running until EOF/Ctrl-C")


    (options, args) = parser.parse_args()
    if len(args) != 0:
        parser.print_help()
        raise SystemExit(1)

    return (options)

if __name__ == '__main__':
    (options) = get_options()
    tb = top_block(options)
    tb.start()
    try:
        if options.interactive:
            input('Press Enter to quit: ')
            tb.stop()
        tb.wait()
    except KeyboardInterrupt:
        tb.stop()
        tb.wait()
