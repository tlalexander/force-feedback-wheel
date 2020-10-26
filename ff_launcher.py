import multiprocessing as mp
import ff_ex_extended
import motor_test_wheel



pipe1, pipe2 = mp.Pipe()

motor_proc = mp.Process(target=motor_test_wheel.main, args=(pipe1,))
motor_proc.start()

ff_proc = mp.Process(target=ff_ex_extended.main, args=(pipe2,))
ff_proc.start()
