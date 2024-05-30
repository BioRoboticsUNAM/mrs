#!/usr/bin/env python
from __future__ import print_function

import sys

if sys.version_info[0] == 2:
	print("Python2 detected.")
	from sn27 import main
	main()
elif sys.version_info[0] == 3:
	print("Python3 detected.")
	from sn3x import main
	main()
