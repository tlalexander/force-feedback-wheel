
"""
MIT LICENSE:
https://mit-license.org/
Copyright © 2020 TLA
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
"""

import multiprocessing as mp
import ff_ex_extended
import motor_test_wheel


pipe1, pipe2 = mp.Pipe()

motor_proc = mp.Process(target=motor_test_wheel.main, args=(pipe1,))
motor_proc.start()

ff_proc = mp.Process(target=ff_ex_extended.main, args=(pipe2,))
ff_proc.start()
