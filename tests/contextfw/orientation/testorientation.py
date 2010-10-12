#!/usr/bin/env python
##
## Copyright (C) 2009-2010 Nokia Corporation
##
## Contact: Jean-Luc Lamadon <jean-luc.lamadon@nokia.com>
##          Matias Muhonen <ext-matias.muhonen@nokia.com>
##          Tapio Rantala <ext-tapio.rantala@nokia.com>
##          Lihan Guo <lihan.guo@digia.com> 
##
## This file is part of Sensord.
##
## Sensord is free software; you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License
## version 2.1 as published by the Free Software Foundation.
##
## Sensord is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
## Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public
## License along with Sensord.  If not, see <http://www.gnu.org/licenses/>.
##
##

import sys
import unittest
import os
import commands
import string
from string import Template

import time
import signal

from ContextKit.cltool import CLTool

def timeoutHandler(signum, frame):
    raise Exception('Tests have been running for too long')

class Orientation(unittest.TestCase):

    def setUp(self):
        self.fpath = "/tmp/fakedsensors/accelerometer"
        self.datafaker = "/usr/bin/datafaker"
        self.context_client_edge = CLTool("context-listen", "Screen.TopEdge")
        self.context_client_cover = CLTool("context-listen", "Screen.IsCovered")


    def tearDown(self):
        self.context_client_edge.atexit()
        self.context_client_cover.atexit()

    def testOrientation(self):

        # Top side up
        os.system("echo 60 960 18 | " + self.datafaker + " " + self.fpath)
        self.assert_(self.context_client_edge.expect('Screen.TopEdge = QString:"top"'))
        time.sleep(0.9)

        # Right side up
        os.system("echo 936 162 180 | " + self.datafaker + " " + self.fpath)
        self.assert_(self.context_client_edge.expect('Screen.TopEdge = QString:"right"'))
        time.sleep(0.9)

        # Bottom up
        os.system("echo 72 -990 -162 | " + self.datafaker + " " + self.fpath)
        self.assert_(self.context_client_edge.expect('Screen.TopEdge = QString:"bottom"'))
        time.sleep(0.9)

        # Left side up
        os.system("echo -954 -90 -36 | " + self.datafaker + " " + self.fpath)
        self.assert_(self.context_client_edge.expect('Screen.TopEdge = QString:"left"'))
        time.sleep(0.9)

        # On the table
        os.system("echo -36 -90 953 | " + self.datafaker + " " + self.fpath)
        self.assert_(self.context_client_cover.expect('Screen.IsCovered = bool:false'))
        time.sleep(0.9)

        # On the table upside down
        os.system("echo 270 216 -972 | " + self.datafaker + " " + self.fpath)
        self.assert_(self.context_client_cover.expect('Screen.IsCovered = bool:true'))


if __name__ == "__main__":
    sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 1)
    signal.signal(signal.SIGALRM, timeoutHandler)
    signal.alarm(10)
    unittest.main()
