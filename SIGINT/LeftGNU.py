#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Signaltest
# Generated: Wed Mar 10 23:26:04 2021
##################################################

from distutils.version import StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from PyQt5 import Qt
from PyQt5 import Qt, QtCore
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import qtgui
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import osmosdr
import sip
import sys
import time
from gnuradio import qtgui


class SignalTest(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Signaltest")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Signaltest")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "SignalTest")

        if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
            self.restoreGeometry(self.settings.value("geometry").toByteArray())
        else:
            self.restoreGeometry(self.settings.value("geometry", type=QtCore.QByteArray))

        ##################################################
        # Variables
        ##################################################
        self.samp_rate_0 = samp_rate_0 = 2000000
        self.freq = freq = 2405000000

        ##################################################
        # Blocks
        ##################################################
        self.qtgui = qtgui.sink_c(
        	1024, #fftsize
        	firdes.WIN_HAMMING, #wintype
        	freq, #fc
        	samp_rate_0, #bw
        	"", #name
        	True, #plotfreq
        	True, #plotwaterfall
        	True, #plottime
        	True, #plotconst
        )
        self.qtgui.set_update_time(1.0/10)
        self._qtgui_win = sip.wrapinstance(self.qtgui.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_win)

        self.qtgui.enable_rf_freq(False)



        self.file_source_block_0 = blocks.file_source(gr.sizeof_gr_complex*1, '/home/meach/Documents/Capstone/GNUradio/Sig2405G2SR/shortLeft2405G2M.dll', True)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vcc((6, ))
        self.Transmit = osmosdr.sink( args="numchan=" + str(1) + " " + '' )
        self.Transmit.set_sample_rate(samp_rate_0)
        self.Transmit.set_center_freq(freq, 0)
        self.Transmit.set_freq_corr(0, 0)
        self.Transmit.set_gain(10, 0)
        self.Transmit.set_if_gain(20, 0)
        self.Transmit.set_bb_gain(16, 0)
        self.Transmit.set_antenna('', 0)
        self.Transmit.set_bandwidth(0, 0)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.Transmit, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.qtgui, 0))
        self.connect((self.file_source_block_0, 0), (self.blocks_multiply_const_vxx_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "SignalTest")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_samp_rate_0(self):
        return self.samp_rate_0

    def set_samp_rate_0(self, samp_rate_0):
        self.samp_rate_0 = samp_rate_0
        self.qtgui.set_frequency_range(self.freq, self.samp_rate_0)
        self.Transmit.set_sample_rate(self.samp_rate_0)

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.qtgui.set_frequency_range(self.freq, self.samp_rate_0)
        self.Transmit.set_center_freq(self.freq, 0)


def main(top_block_cls=SignalTest, options=None):
    
    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    
    tb.start()
    # tb.show()
    time.sleep(1.5)
    def quitting():
        tb.stop()
        tb.wait()

    qapp.aboutToQuit.connect(quitting)
    # This combined with the tb.show() would bring up the GUI which we don't need or want since we just want to send the signal and be done with it
    # qapp.exec_()
    
    
    

if __name__ == '__main__':
    main()


